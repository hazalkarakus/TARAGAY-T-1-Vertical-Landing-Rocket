#!/usr/bin/env python3
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8", errors="strict")


def require(text: str, token: str, label: str) -> None:
    if token not in text:
        raise AssertionError(label)


config = read("App/Common/app_config.h")
version = read("App/Common/app_version.h")
project = read(".project")
cproject = read(".cproject")
app = read("App/app.c")
tasks = read("App/Core/Tasks/app_tasks.c")
irq = read("Core/Src/stm32f4xx_it.c")
adaptive = read("App/Services/NeedleValveIntegrationTest/needle_valve_integration_test.c")
autonomous = read("App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c")
autonomous_h = read("App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.h")
preflight = read("App/Services/PreflightTrigger/preflight_trigger.c")

checks = (
    (config, "#define APP_NEEDLE_AUTONOMOUS_ACTUATOR_MODE              1U", "autonomous mode"),
    (config, "#define APP_NEEDLE_FOUR_TURN_TEST_MODE                   0U", "four-turn bench disabled"),
    (config, "#define APP_NEEDLE_RAW_CHARACTERIZATION_MODE             0U", "raw bench disabled"),
    (config, "#define APP_NEEDLE_ADC_DIAGNOSTIC_MODE                   0U", "ADC diagnostic disabled"),
    (config, "#define APP_V55_UART_RX_COMMANDS_ENABLED                0U", "UART RX disabled"),
    (version, "8.19M-P112R8-PRODUCTION-AUTONOMOUS-INTEGRATION", "version"),
    (project, "TGY_V8_19M_P112R8_PRODUCTION_AUTONOMOUS_INTEGRATION", "CubeIDE project"),
    (cproject, "${workspace_loc:/TGY_V8_19M_P112R8_PRODUCTION_AUTONOMOUS_INTEGRATION}/Debug", "CubeIDE debug path"),
    (cproject, "${workspace_loc:/TGY_V8_19M_P112R8_PRODUCTION_AUTONOMOUS_INTEGRATION}/Release", "CubeIDE release path"),
    (app, "NeedleValveAutonomousControl_Init();", "supervisor init"),
    (tasks, "GeneratedFlightControl_GetStatus();", "generated FC source"),
    (tasks, "NeedleValveAutonomousControl_SubmitCommand(command)", "200 Hz production command route"),
    (tasks, "NeedleValveAutonomousControl_RevokeAuthorization();", "authorization revoke route"),
    (irq, "NeedleValveAutonomousControl_TimerTickISR();", "TIM7 supervisor"),
    (adaptive, "P112R5_MAX_POWERED_LIMIT_MS             2600U", "powered hard limit"),
    (autonomous, "P112R5_COMMAND_TIMEOUT_MS               50UL", "command watchdog"),
    (preflight, "NeedleValveAutonomousControl_CaptureClosedReference();", "motorless reference"),
    (app, "NVIC_DisableIRQ(TIM7_IRQn);", "SD boot TIM7 NVIC isolation"),
    (app, "TIM7->DIER &= ~TIM_DIER_UIE;", "SD boot TIM7 UIE isolation"),
    (app, "TIM7->CR1 &= ~TIM_CR1_CEN;", "SD boot TIM7 stop"),
    (app, "NVIC_ClearPendingIRQ(TIM7_IRQn);", "TIM7 pending clear"),
    (app, "TIM7->CNT = 0UL;", "TIM7 clean restart phase"),
    (app, "SDLogger_Update();", "runtime SD service retained"),
)

for text, token, label in checks:
    require(text, token, label)

# R7 bench command path must be completely absent from production source/config.
for text, label in ((config, "config"), (tasks, "tasks"), (autonomous, "autonomous c"), (autonomous_h, "autonomous h")):
    if "P112R7" in text or "BenchValidationUpdate" in text or "SINGLE_SHOT_BENCH" in text:
        raise AssertionError(f"R7 bench path still present in {label}")

if "P112R4" in adaptive or "P111_AutoStartService" in adaptive:
    raise AssertionError("fixed P112R4 boot movement still present")

host_binary = Path("/tmp/p112r8_autonomous_host_test")
subprocess.run(
    [
        "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-Itools/p112r5_host_stub", "-IApp",
        "tools/p112r5_autonomous_host_test.c",
        "App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c",
        "-o", str(host_binary),
    ],
    cwd=ROOT,
    check=True,
)
subprocess.run([str(host_binary)], cwd=ROOT, check=True)

print("P112R8 static production architecture checks: PASS")
print("P112R8 autonomous supervisor host behavior test: PASS")
print("P112R8 R7-bench-path absence check: PASS")
print("P112R8 SD-boot TIM7 isolation markers: PASS")
