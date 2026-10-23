# OpenFFBoard-XDrive

Fork OpenFFBoard (MIT, Yannick Richter) z portem na płytkę **MKS XDrive Mini V1.0** (STM32F405RG +
DRV8301). Gałąź robocza: `mks-xdrive-mini`. Kontekst projektu: `../CLAUDE.md`. Aplikacja: `../wheel-control`.

## Budowanie – tylko ten target

```bash
export PATH=/c/Users/carsb/tools/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin:/c/Users/carsb/tools/xpack-windows-build-tools-4.4.1-3/bin:$PATH
make -C Firmware -j8 MCU_TARGET=F405RG_XDRIVE
```

Wynik: `Firmware/build/OpenFFBoard_F405RG_XDRIVE.hex`. **Nie buduj innych targetów** (F407VG, F411RE, …)
ani nie dodawaj ich do macierzy CI (`.github/workflows/build-firmware.yml`).
Wgrywanie: przycisk *Tryb DFU* w DD Control (`sys.dfu`) → STM32CubeProgrammer po USB.

Symulacja sterownika na PC (WSL, g++ ≥ 10): `make -C Firmware/Targets/F405RG_XDRIVE/sim run` –
uruchom po każdej zmianie w `MotorXDrive.cpp`.

## Co jest nasze

- `Firmware/Targets/F405RG_XDRIVE/` – target: `main.h` (piny), `main.c` (zegary 168 MHz, TIM1 PWM
  20 kHz + TRGO=OC4REF → ADC2/3 injected, TIM2 hamulec, TIM3 enkoder, TIM4 → ADC1, SPI3 DRV8301,
  USB bez VBUS), `stm32f4xx_it.c`, `target_constants.h`, `README.md` (uruchomienie krok po kroku,
  tabela błędów), `sim/`. F405 kompilowany jako `STM32F407xx`.
- `Firmware/FFBoard/UserExtensions/Src/MotorXDrive.cpp` (+ `.h`) – prądowe FOC, kalibracja
  (R, L, kierunek, offset), środek z indeksu Z, autotest, rezystor hamujący, zabezpieczenia.
  Komendy klasy `xdrive.0.*`; stany 0–7 i bity błędów opisane w README targetu.
- Małe zmiany w upstream (zawsze za `#ifdef` z `target_constants.h`): rejestracja w
  `Axis::axis1_drivers` (id 13, `CLSID_MOT_XDRIVE`), `DEFAULT_DRVTYPE`/`FORCE_DEFAULT_DRVTYPE`,
  `ENDSTOP_FULL_TORQUE`/`ENDSTOP_DAMPING` w `Axis.cpp`, `Flash_WriteBegin/End` w `flash_helpers.cpp`,
  adresy flash `0x420+` w `scripts/memory_map.csv` (→ `generate_memory.py`).

## Pułapki

- Oś oferuje tylko sterowniki z `Axis::axis1_drivers` (nie `MotorDriver::all_drivers`) – CI to sprawdza.
- Przy zapisie do flash CPU stoi – mostek jest na ten czas wyłączany (hooki `Flash_Write*`).
- Konfiguracja DRV8301 po SPI tylko przy poprawnym napięciu zasilania (inaczej nFAULT).
- Kod upstream zostawiamy w stylu upstream (taby, komentarze po angielsku); nowe zachowanie za
  define'ami targetu, żeby było widać, co zmieniliśmy.
