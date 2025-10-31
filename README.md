# realtime-frequency-meter

Real-time frequency meter for the TM4C1294 LaunchPad (EK‑TM4C1294XL). The application measures the frequency of a digital external signal with TIMER0A edge counting and prints results on UART0. The firmware uses CMSIS‑RTOS2 (RTX) and TivaWare.

I include auxiliary RTOS examples in the repository to demonstrate threads, timers, semaphores, and flags.


**Key Features**
- Adjust the measurement window: 1 s (prints in Hz) or 1 ms (prints in kHz).
- Toggle the window with the user button SW1 (PJ0).
- Feed the input signal to `PL4 (T0CCP0)` using `TIMER0A` in `CAP_COUNT_UP` mode.
- Print results on `UART0` at 115200 8N1 (ICDI Virtual COM on `PA0/PA1`).
- Run separate RTOS threads for measurement, UART, and button handling.


**Hardware**
- Board: TI TM4C1294NCPDT LaunchPad (EK‑TM4C1294XL).
- Relevant pins:
  - Signal input: `PL4 / T0CCP0` (BoosterPack header).
  - Button SW1: `PJ0` (with internal pull‑up enabled).
  - UART0: `PA0 (U0RX)`, `PA1 (U0TX)` — via ICDI/USB at 115200 8N1.
  - LEDs: `D1=PN1`, `D2=PN0`, `D3=PF4`, `D4=PF0` (used in examples).


**Main Project (Meter)**
- Path: `tarefas/src/main.c:1`
- How it works:
  - Configure `TIMER0A` to count rising edges on `T0CCP0 (PL4)`.
  - Configure `TIMER1A` to generate the measurement window (1 s or 1 ms).
  - The measurement thread reads the count, clears the counter at the end of each window, and notifies the UART thread according to the current scale.
  - The UART thread prints values in Hz (1 s window) or kHz (1 ms window).
  - The button thread debounces SW1 and toggles the window.
- Useful code references:
  - Timer and UART setup in `tarefas/src/main.c:1`.
  - LED abstraction in `tarefas/src/driverleds.c:1`.


**Build and Flash (IAR)**
1. Requirements:
   - IAR Embedded Workbench for ARM 8.x.
   - ICDI drivers for the LaunchPad.
2. Open the workspace `CMSIS_RTOS2_IAR8_Tiva.eww:1` in IAR.
3. Build the projects (or use Build All) in this order:
   - `driverlib/driverlib.ewp:1` (TivaWare DriverLib)
   - `CMSIS/RTOS2/RTX/Library/IAR/IDE/RTX_CM.ewp:1` (RTX library)
   - `tarefas/tarefas.ewp:1` (meter application)
4. Connect the board over USB (DEBUG/ICDI port) and select the `Debug` target.
5. Use “Download and Debug” to program the board.


**Signal Wiring**
- Connect the signal source to `PL4 (T0CCP0)` and to the board GND.
- Drive 3.3 V logic levels. Condition higher voltages before the pin.
- Expect a range from a few Hz to a few MHz, limited by hardware and signal quality.


**Usage**
1. Open a serial terminal on the board COM port (ICDI), `115200 8N1`.
2. Reset the board. The firmware starts with a 1 s window (prints in Hz each second).
3. Press `SW1` (PJ0) to switch to a 1 ms window and print in kHz.
4. Press `SW1` again to return to the 1 s window.


**Architecture**
- `thread_calcFreq`: waits for the timer flag, reads `TIMER0A` count, clears the counter, and decides when to notify the UART.
- `thread_uart`: prints the formatted value ("N Hz" or "N kHz").
- `thread_botao`: debounces `PJ0` and toggles the window (1 s ↔ 1 ms).
- `timerInterrupHandler`: clears `TIMER1A` interrupt and wakes the measurement thread.


**Repository Structure**
- `tarefas/` — Main project (frequency meter)
  - Application code: `tarefas/src/main.c:1`, `tarefas/src/driverleds.c:1`.
- `sinalizador/`, `temporizador/`, `prodcons/`, `blinky/` — RTOS examples
  - Examples: `sinalizador/src/main.c:1`, `temporizador/src/main.c:1`, `prodcons/src/main.c:1`, `blinky/src/main.c:1`.
- `driverlib/`, `inc/` — TivaWare (DriverLib and device headers).
- `CMSIS/` — CMSIS‑RTOS2 (RTX) and examples.


**Notes and Limitations**
- Accuracy depends on the MCU clock and the input signal quality.
- The kHz mode uses 1 ms windows; the printed value equals counts per ms (1 count = 1 kHz).
- The `PL4` input has no analog filtering; add signal conditioning when needed.
- The firmware reads `TAR` and clears `TAV` (see `tarefas/src/main.c:1`); many applications read `TAV`, but this approach suits the project goals on this device.


**Troubleshooting**
- No terminal output: check the correct COM port, `115200 8N1`, ICDI drivers, and power.
- No counting: verify wiring on `PL4/T0CCP0` and common GND; confirm logic levels.
- Build errors: ensure include paths for `inc/` and `driverlib/`; use the provided workspace.


**Credits**
- I built this on CMSIS‑RTOS2 (RTX) and Texas Instruments TivaWare libraries.


**Tech Stack**
- MCU/Board: TM4C1294NCPDT LaunchPad (EK‑TM4C1294XL).
- Language: C (IAR toolchain).
- RTOS: CMSIS‑RTOS2 (Keil RTX5) for threads, flags, semaphores, and timers.
- HAL/Drivers: TivaWare DriverLib and CMSIS core/device headers.
- Console: UARTStdio (TivaWare utils) over UART0.
- Toolchain/IDE: IAR Embedded Workbench for ARM 8.x with C‑SPY/ICDI.
