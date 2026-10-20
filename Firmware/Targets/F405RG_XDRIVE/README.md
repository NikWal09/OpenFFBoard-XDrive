# OpenFFBoard na MKS XDrive Mini V1.0 (target `F405RG_XDRIVE`)

Kierownica Direct Drive: płytka MKS XDrive Mini (STM32F405RGT6 + DRV8301), silnik z hoverboardu
(15 par biegunów), enkoder MT6701 w trybie ABZ (4096 CPR) z indeksem Z, zasilacz Dell 19,5 V / 240 W,
rezystor hamujący na złączu AUX.

Target uruchamia OpenFFBoard jako **FFB Wheel (1 oś)** z własnym sterownikiem silnika
**`MotorXDrive`** (`FFBoard/UserExtensions/Src/MotorXDrive.cpp`): prądowe FOC 20 kHz,
automatyczna kalibracja, środek względem impulsu Z, autotest, rezystor hamujący.
Konfiguracja: OpenFFBoard Configurator albo DD Control (port COM, USB `1209:FFB0`).

> **Firmware nie był jeszcze uruchamiany na prawdziwej płytce.** Logika sterownika jest sprawdzona
> w symulacji (`sim/`), ale pierwsze uruchomienie rób dokładnie według kroków poniżej.
> Silnik ma ok. 15 Nm – przy błędzie potrafi gwałtownie szarpnąć kierownicą.

## Kompilacja

Toolchain: Arm GNU / xPack `arm-none-eabi-gcc` 15 i `make`.

```bash
make -C Firmware -j8 MCU_TARGET=F405RG_XDRIVE
```

Wynik: `Firmware/build/OpenFFBoard_F405RG_XDRIVE.hex` / `.bin`. CI GitHuba buduje ten target razem
z pozostałymi.

## Wgranie (USB DFU)

1. Odłącz zasilacz, zostaw tylko USB-C.
2. Przytrzymaj **S2 (BOOT0)**, naciśnij i puść **S1 (RESET)**, puść BOOT0.
3. STM32CubeProgrammer → *USB* → *Connect* → wgraj `.hex` (albo `.bin` od `0x08000000`) → *Disconnect*.
4. Naciśnij RESET. W Windows pojawi się kontroler gier i port COM „OpenFFBoard”.

Kolejne aktualizacje: przycisk *Tryb DFU* w DD Control (komenda `sys.dfu`) zamiast przycisków na płytce.
Pierwsze wgranie kasuje ustawienia FFBeast – OpenFFBoard używa innego układu pamięci.

## Pierwsze uruchomienie – krok po kroku

Na każdym kroku sprawdzaj *Baza i silnik → Diagnostyka XDrive* w DD Control (albo komendy
`xdrive.0.state?`, `xdrive.0.errors?` w konsoli Configuratora). **Awaryjnie: wyciągnij zasilacz.**

| # | Co zrobić | Czego oczekiwać |
|---|---|---|
| 1 | Tylko USB, bez zasilacza | Stan **Start (0)**, napięcie ~0 V, błędy 0. Mostek wyłączony. |
| 2 | Obróć kierownicę ręką o pełny obrót | Kąt na pulpicie się zmienia, *Indeks Z wykryty* = Tak. |
| 3 | Ustaw *Limit prądu* **3000 mA**, *Limit napięcia* **4000 mV**, *Moc* (axis.power) ~**10%**, zapisz | – |
| 4 | Podłącz zasilacz 19,5 V, **ręce z dala od kierownicy** | Stan: Pomiar offsetów → **Bezczynny (2)**, błędy **512** (brak kalibracji), napięcie ~19,5 V. |
| 5 | *Kalibracja silnika* | ~12 s: wirnik ustawia się (lekki ruch), krótkie brzęczenie (pomiar L), obrót o kilka stopni, ~48° w przód i z powrotem. Na końcu stan **Praca (5)**. |
| 6 | Sprawdź wyniki | Rezystancja ~100–300 mΩ, indukcyjność ~200–1000 µH, *Kontrola biegunów/CPR* **950–1050 ‰**. |
| 7 | Moc 10%, *Sprężyna bezczynności* ~50%. Wychyl kierownicę ręką | Kierownica **wraca do środka**. Jeśli **odpycha od środka** – natychmiast odłącz zasilacz i zgłoś (błąd kierunku). |
| 8 | *Test obrotu* (puść kierownicę) | +360°, −720°, +360°, powrót do zera, bez błędów. |
| 9 | Ustaw kierownicę prosto → *Wycentruj* → *Zapisz w bazie* | Pozycja = 0°. Po następnym włączeniu środek wróci po minięciu impulsu Z (włączaj w zakresie ±180° od środka). |
| 10 | Stopniowo zwiększaj *Limit prądu* (5 → 8 → 10 A) i *Moc* | Zasilacz 12,3 A; prąd fazy może być wyższy od prądu z zasilacza. |

Po każdej udanej zmianie: *Zapisz w bazie* (`sys.save`). Na czas zapisu do flash mostek jest na
chwilę wyłączany – to celowe.

## Diagnostyka

`xdrive.0.errors?` (bity, kasowanie: `xdrive.0.errors=0`):

| Bit | Znaczenie | Co sprawdzić |
|---|---|---|
| 1 | DRV8301 nFAULT (nadprąd, przegrzanie, za niskie napięcie bramek) | `xdrive.0.drvstatus?`, zwarcia faz, limit prądu |
| 2 | DRV8301 nie odpowiada po SPI | lutowania, zasilanie z zasilacza (DRV jest zasilany z szyny) |
| 4 | Offsety wzmacniaczy prądu poza zakresem | wzmacniacze DRV8301, VREF |
| 8 | Nadnapięcie > 25 V | rezystor hamujący, zasilacz |
| 16 | Napięcie < 10 V (nie blokuje, tylko brak momentu) | zasilacz |
| 32 | Rezystor hamujący włączony > 5 s | czy rezystor jest podłączony, wartość ok. 2–10 Ω |
| 64 | Kalibracja nie osiągnęła prądu | przewody faz, *Limit napięcia* za niski |
| 128 | Ruch enkodera niezgodny z biegunami/CPR | *Pary biegunów* (15), *CPR* (4096), mocowanie magnesu |
| 256 | Rezystancja poza zakresem 5 mΩ–5 Ω | przewody faz |
| 512 | Brak kalibracji (informacja) | uruchom *Kalibrację* |
| 1024 | Programowy limit prądu przekroczony | *Limit prądu*, pasmo regulatora |
| 2048 | Autotest przerwany (pozycja odjechała > 90°) | coś blokuje kierownicę, za mały limit prądu |

## Założenia do zweryfikowania na sprzęcie

- **Pomiar prądu:** kalibracja sama wykrywa znak wzmacniaczy i to, który kanał ADC widzi fazę B/C.
  Zakładane wzmocnienie DRV8301 = 40 V/V (ustawiane po SPI) i boczniki 0,5 mΩ.
- **Rezystor hamujący:** zakładam, że rezystor obciąża górny tranzystor AUX (jak w ODrive v3, dolny
  służy tylko do ładowania bootstrapu). Przy szybkim hamowaniu `xdrive.0.brake?` > 0 – jeśli rezystor
  wtedy się nie grzeje, a napięcie rośnie do błędu 8, rezystor jest po drugiej stronie półmostka.
- **Czas martwy:** 100 ns z MCU + wewnętrzny DRV8301; błąd napięcia jest mierzony przy wyrównaniu
  i kompensowany.
- **Kierunek indeksu Z:** zbocze opadające na PC9.

## Komendy `xdrive.0.*`

`poles`, `cpr`, `vlimit` (mV), `ilimit` (mA), `mode` (1 FOC / 0 napięciowy), `bw` (rad/s),
`index` (0/1), `sensorDirection` (−1/1), `invert`, `cal`, `test`, oraz odczyty: `state`, `errors`,
`vbus` (mV), `iq` (mA), `res` (0,1 mΩ), `ind` (µH), `offset`, `brake` (%), `center`,
`indexfound`, `calratio` (‰), `drvstatus`.

## Symulacja na PC

`sim/` kompiluje prawdziwy `MotorXDrive.cpp` z atrapami rejestrów STM32 i modelem silnika
(PMSM, czas martwy, szum ADC, enkoder z indeksem). Sprawdza 16 kombinacji okablowania i polaryzacji
czujników oraz stany awaryjne (zanik zasilania, hamowanie, nFAULT, zła liczba biegunów).

```bash
make -C Firmware/Targets/F405RG_XDRIVE/sim run   # Linux / WSL, g++ >= 10
```

## Pliki targetu

- `Core/Inc/main.h` – mapa pinów (bez CubeMX; projekt `.ioc` nie istnieje).
- `Core/Src/main.c` – zegary 168 MHz, TIM1/2/3/4/13, ADC1–3, SPI3, USB bez VBUS.
- `Core/Inc/target_constants.h` – funkcje OpenFFBoard włączone dla tej płytki.
- F405 jest kompilowany jako `STM32F407xx` (ten sam krzem, DEVID 0x413).
