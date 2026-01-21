# CE224 - STM32 Audio Reactive LED Controller

This project utilizes the **STM32F407VET6** microcontroller to drive two separate **APA102 LED strips** with real-time audio visualization (FFT). **It controls the LED strips using USART and SPI communication protocols to ensure high-speed and synchronized data transmission.** The system also features a cyberpunk-themed Web Dashboard for remote control via Bluetooth/Web Serial API.

## 🛠 Hardware Specifications
- **MCU Board:** STM32F407VET6 (Black Board).
- **LEDs:** 2 x APA102 LED Strips (or SK9822).
- **Audio Input:** **MAX9814** Microphone Module (with AGC).
- **Control Input:** 1 x Potentiometer (Controls Global Brightness).
- **Connectivity:** HC-05 Bluetooth Module.
- **User Interface:** 2 x Physical Buttons + Web Dashboard.

## 🔌 Pinout Configuration

### 1. LED Drivers (Dual Protocol)
| LED Strip | Protocol | Data Pin | Clock Pin | Note |
| :--- | :--- | :--- | :--- | :--- |
| **Strip 1** | **USART6** (Sync) | **PC6** (TX) | **PC8** (CK) | *Synchronous Master Mode* |
| **Strip 2** | **SPI3** | **PB5** (MOSI) | **PC10** (SCK)| *Transmit Only Master* |

### 2. Sensors & Analog Inputs (ADC)
| Component | STM32 Pin | ADC Channel | Function |
| :--- | :--- | :--- | :--- |
| **MAX9814** | **PA0** | **ADC1_IN0** | Audio Signal Input (Microphone) |
| **Potentiometer**| **PA1** | **ADC2_IN1** | **Global Brightness Control** |

### 3. Controls & Communication
| Component | STM32 Pin | Function |
| :--- | :--- | :--- |
| **Button 1** | **PE1** | Change Mode for **UART LED** (Strip 1) |
| **Button 2** | **PE0** | Change Mode for **SPI LED** (Strip 2) |
| **HC-05** | **PA2** (TX) | Transmit data to Web/PC |
| | **PA3** (RX) | Receive commands from Web/PC |

## 📁 Project Structure

The project follows a **Modular Header Implementation** pattern. All functional logic is encapsulated within the `Core/Inc` directory, while `Core/Src` is kept minimal for high-level execution.

- **Core/Src/**
    - `main.c`: **Application Entry Point.** It initializes peripherals (HAL) and acts as the orchestrator, calling high-level functions defined in the headers (e.g., `button_scan()`, `fft_process()`, `led_show()`).

- **Core/Inc/** (Functional Logic & Implementations)
    - `Config.h`: Global system configuration (Pin definitions, Buffer sizes, Thresholds).
    - `FFT.h`: Contains the Fast Fourier Transform algorithms and audio processing logic.
    - `SPI_LED.h`: Driver logic for the SPI-connected LED strip.
    - `UART_LED.h`: Driver logic for the USART-connected LED strip.
    - `button.h`: Handling button inputs, debouncing, and state changes.
    - `knob.h`: Logic for reading and smoothing Potentiometer (ADC) values.
    - `bluetooth.h`: UART interrupt handling and command parsing for HC-05.
    - `effect.h`: Definitions of visual effects and color palettes.

- **Web/**
    - `index.html`: The dashboard UI (HTML/JS/CSS).
    - `connected.mp3`: Audio notification file.

## 🚀 Usage Guide
1. **Power Up:**
   - Supply **5V** to the LED Strips (External Power Supply).
   - Supply power to the STM32 via USB.
   - **IMPORTANT:** Connect the GND of the external power supply to the STM32 GND.
2. **Operation:**
   - The system starts with the default effect.
   - Adjust the **Potentiometer (PA1)** to change the **Brightness** of the LEDs.
   - Press **PE0** or **PE1** to cycle through visual effects.
3. **Web Control:**
   - Open `Web/index.html` in Chrome or Edge.
   - Click **Connect** and select the HC-05 COM Port.
   - Listen for the "Connected" sound and use the dashboard to control the system.
  
## 👥 Authors & Contributors

* **Nguyen Trong Bao Duy** - [Thorlion](https://github.com/Thorlion)
* **Nguyen Dang Phuong Duy** - [DuyNDP](https://github.com/DuyNDP)
* **Vu Dai Duong** - [VuDaiDuong_325](https://github.com/VuDaiDuong-325)
*  **Phan Thanh Duy**
