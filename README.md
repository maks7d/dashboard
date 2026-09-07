# Dashboard Motorsport — STM32H743

Firmware embarqué pour un dashboard de sport auto/moto/karting. Affichage des temps au tour, données IMU, GPS, gestion batterie et retour visuel via LEDs adressables.

---

## Installation et setup développeur

### Prérequis logiciels

#### 1. VS Code (Optionnel)
Télécharger sur [code.visualstudio.com](https://code.visualstudio.com).

Extensions obligatoires (installer via `Ctrl+Shift+X`) :

| Extension | ID | Rôle |
|---|---|---|
| C/C++ | `ms-vscode.cpptools` | IntelliSense |
| CMake Tools | `ms-vscode.cmake-tools` | Build intégré |
| Cortex-Debug | `marus25.cortex-debug` | Flash et debug ST-Link |

#### 2. ARM GNU Toolchain
Télécharger la dernière version pour votre OS (Windows, Linux, macOS) de **arm-none-eabi** depuis [developer.arm.com](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads).

Installer et vérifier :
```bash
arm-none-eabi-gcc --version
```

#### 3. CMake
Télécharger depuis [cmake.org](https://cmake.org/download/) (version 3.22 minimum) ou avec apt(linux) ou brew(macOS).

```bash
cmake --version
```

#### 4. Ninja pour compiler plus rapidement (Optionnel)
```bash
winget install Ninja-build.Ninja
```

Vérifier après redémarrage du terminal :
```bash
ninja --version
```
---

### Cloner et compiler

```bash
git clone <url-du-repo>
cd dashbaord

# sur Linux/MacOS
chmod +x ./build.sh
./build.sh
```

Le fichier compilé est généré dans `build/Debug` sous différents formats :
```
build/Debug/dashboard.elf
build/Debug/dashboard.hex
build/Debug/dashboard.bin
```

---

## Architecture du projet

### Structure des fichiers

```
dashbaord/
│
├── Dash.ioc                        # Projet STM32CubeMX — source de vérité des pins et périphériques
│
├── CMakeLists.txt                  # Point d'entrée CMake (ajouter ses propres fichiers ici)
├── CMakePresets.json               # Presets Debug / Release (Ninja)
├── STM32H743XX_FLASH.ld            # Linker script — carte mémoire Flash/RAM
├── startup_stm32h743xx.s           # Startup assembleur — initialise la pile et appelle main()
│
├── cmake/
│   ├── gcc-arm-none-eabi.cmake     # Toolchain ARM (flags, compilateur, linker)
│   └── stm32cubemx/
│       └── CMakeLists.txt          # Sources et includes générés par CubeMX (ne pas modifier)
│
├── Core/
│   ├── Inc/                        # Headers
│   │   ├── main.h                  # Defines globaux des pins (POWER_HOLD_PIN, etc.)
│   │   ├── stm32h7xx_hal_conf.h    # Active/désactive les modules HAL
│   │   └── [periph].h              # Headers init périphériques générés par CubeMX
│   │
│   └── Src/                        # Sources
│       ├── main.c                  # Point d'entrée applicatif — FICHIER PRINCIPAL
│       ├── stm32h7xx_it.c          # Handlers d'interruptions
│       ├── stm32h7xx_hal_msp.c     # Init bas niveau des périphériques (GPIOs AF, DMA)
│       ├── gpio.c                  # Init GPIO — modifié manuellement pour power hold
│       ├── tim.c                   # Init TIM2 (input capture lap detection)
│       ├── spi.c                   # Init SPI1 (BMI270) et SPI2 (WS2812B)
│       ├── usart.c                 # Init USART1/2 (GPS NEO-M9N)
│       ├── i2c.c                   # Init I2C1 (BQ25887)
│       ├── adc.c                   # Init ADC2 (capteurs analogiques)
│       ├── syscalls.c              # Stubs newlib (_write, _read, etc.)
│       └── system_stm32h7xx.c      # Init système CMSIS (appelée avant main)
│
└── Drivers/
    ├── STM32H7xx_HAL_Driver/       # Librairie HAL ST — ne jamais modifier
    └── CMSIS/                      # Headers CMSIS Cortex-M7 et STM32H7 — ne jamais modifier
```

---

### Règle fondamentale : zones USER CODE

CubeMX écrase les fichiers générés à chaque régénération, **sauf le contenu des zones** `USER CODE` :

```c
/* USER CODE BEGIN xxx */
// Ton code ici → survivra à une régénération CubeMX
/* USER CODE END xxx */
```

Tout code écrit en dehors de ces zones dans les fichiers générés sera perdu lors d'une régénération. Le seul fichier qui ne sera jamais régénéré est `CMakeLists.txt` (marqué "generated only once").

---

### Ajouter ses propres fichiers

1. Créer `Core/Src/mon_driver.c` et `Core/Inc/mon_driver.h`
2. Déclarer la source dans `CMakeLists.txt` :

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/mon_driver.c
)
```

3. L'include path `Core/Inc/` est déjà configuré — `#include "mon_driver.h"` fonctionne directement.

---

### Horloge système

```
HSE (cristal 25 MHz sur PH0/PH1)
    └── PLL1 (M=2, N=64, P=2)
            └── SYSCLK = 400 MHz
                    ├── CPU Cortex-M7  : 400 MHz (D1CPRE /1)
                    ├── AHB / AXI      : 200 MHz (HPRE /2)
                    ├── APB1 (I2C, TIM): 100 MHz (D2PPRE1 /2)
                    ├── APB2 (SPI, UART): 100 MHz (D2PPRE2 /2)
                    └── APB3/4         : 100 MHz (/2)
```

Configurée dans `SystemClock_Config()` dans `Core/Src/main.c`.

---

### Démarrage et power hold

Le circuit d'alimentation repose sur un MOSFET canal P piloté par **PE3**. La logique est :

- `PE3 = HIGH` → MOSFET fermé → circuit alimenté
- `PE3 = LOW`  → MOSFET ouvert → circuit hors tension

#### Séquence de boot dans `main.c`

```
Reset MCU
    │
    ▼ USER CODE BEGIN 1
    Activation clock GPIOE
    PE3 → mode output
    PE3 → HIGH           ← verrouillage PMOS avant tout init
    │
    ▼
    HAL_Init()
    SystemClock_Config()
    MX_GPIO_Init()       ← gpio.c initialise PE3 à HIGH (corrigé)
    ...
    │
    ▼ USER CODE BEGIN 2
    PE3 confirmé HIGH
    MX_GPIO_EXTI_Init()  ← active NVIC pour le bouton power (PG7)
    │
    ▼
    while(1) { ... }     ← PE3 reste HIGH indéfiniment
```

#### Extinction sur appui long (PG7 > 5 secondes)

Implémenté sans polling, uniquement via interruptions :

```
Bouton PG7 appuyé (FALLING edge, pin à GND)
    │
    ▼ EXTI9_5_IRQHandler → HAL_GPIO_EXTI_Callback()
    power_btn_press_tick = HAL_GetTick()
    power_btn_held = 1

    (chaque milliseconde)
    │
    ▼ SysTick → HAL_SYSTICK_Callback()
    si held ET (tick - press_tick) >= 5000ms
        PE3 = LOW → PMOS ouvert → extinction

Bouton relâché avant 5s
    │
    ▼ EXTI9_5_IRQHandler (RISING edge)
    power_btn_held = 0   ← compteur annulé
```

---
