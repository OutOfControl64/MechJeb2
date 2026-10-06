# Balans RCS przy obrocie w MechJeb2: plan wersji pełnej

Plan rozszerzenia RCS Balancera z MechJeb2 tak, żeby **obrót na RCS nie zmieniał prędkości statku**, także przy
komendach mieszanych (obrót + translacja) i przy ciągle zmieniających się komendach kontrolera orientacji. To praca
w repozytorium MechJeba (`MuMech/MechJeb2`), nie w Talarii. Wariant prostszy, niezależny od MechJeba, opisuje
`rcs_balans.md`.

Stan kodu MechJeba: `dev`, commit `a589f1b` (5.10.2026). Numery linii poniżej odnoszą się do tego commita.

## 1. Stan obecny w MechJebie

RCS Balancer („Smart RCS translation”) to `MechJebModuleRCSBalancer` + `RCSSolver` + `RCSSolverThread`.

- **Działa tylko dla translacji.** `MechJebModuleRCSBalancer.cs:207`: „RCS balancing on rotation isn't supported”,
  rotacja jest na sztywno zerem w `RCSSolver.cs:570`.
- **Co klatkę ustawia `thrusterPower`** każdego `ModuleRCS` (`MechJebModuleRCSBalancer.cs:233`), a po translacji
  przywraca wartość oryginalną (`RCSSolver.Thruster.RestoreOriginalForce`). Ograniczników `thrustPercentage` nie rusza.
- **Model dyszy już zna rotację.** `RCSSolver.Thruster.GetThrust(direction, rotation)` (`RCSSolver.cs:44`) liczy
  przepustnicę dyszy jako `clamp01(dot(direction, d) + dot(rotation, −pos × d))`. Komentarz mówi, że to obserwacja
  („The game appears to…”), a nie przepisany kod stock. Położenie `pos` to środek masy **części**, nie dyszy.
- **Funkcja kosztu jest pod translację.** Karze błąd momentu, błąd kierunku pojedynczych dysz (waga 0,005)
  i „marnowany” ciąg względem kierunku translacji (`RCSSolver.cs:156`). Przy samym obrocie kierunek jest zerowy, więc
  człon „waste” dławiłby wszystkie dysze, a prawdziwej siły wypadkowej nikt nie liczy.
- **Wyniki liczy osobny wątek, z cache.** Klucz to kwantyzowany kierunek translacji (`RCSSolverKey`, `RCSSolver.cs:224`),
  rotacji w kluczu nie ma. Gdy wyniku jeszcze nie ma, balancer zeruje ciąg (`MechJebModuleRCSBalancer.cs:219`).
  Optymalizator to `alglib.minbleic` z numerycznym gradientem.
- **Przeliczanie po przesunięciu CoM już jest** (`RCSSolver.cs:489`, próg zależny od momentu bezwładności).
- **Kolejność modułów.** `MechJebCore` woła `Drive` modułów rosnąco po `Priority` (`MechJebCore.cs:358`,
  `ComputerModule.cs:38`). Balancer ma 700, kontroler orientacji 800, więc **balancer działa przed tym, jak kontroler
  orientacji wpisze pitch/yaw/roll** do `FlightCtrlState`. Dla translacji to nie przeszkadza (`MechJebModuleRCSController`
  ma 600), dla rotacji tak.
- **Estymacja momentu dla kontrolera.** `VesselState.UpdateRCSThrustAndTorque` liczy `RCSTorqueAvailable` z bieżącego
  `thrusterPower` (`VesselState.cs:712`), który balancer zmienia co klatkę. Komentarz w kodzie: „Are we missing an
  rcsTorqueAvailable calculation here?”. Ten sam kod pokazuje, jak stock liczy tryb precyzyjny (`useLever`,
  `GetLeverDistance`, `precisionFactor`).

## 2. Cel i założenia

Dla dowolnej komendy z `FlightCtrlState` (translacja `t` = X/Y/Z, obrót `ω` = pitch/roll/yaw, od pilota albo
z kontrolera) dobrać mnożniki `x_i ∈ [0, 1]` dla `thrusterPower` każdej części RCS tak, żeby:

1. przy samym obrocie siła wypadkowa była ≈ 0,
2. przy samej translacji moment był ≈ 0 (dzisiejsze zachowanie),
3. przy komendzie mieszanej siła była równoległa do `t`, a moment do `ω`,
4. ciąg był jak największy (`x` blisko 1), żeby nie tracić sterowności,
5. całość liczyła się **synchronicznie w każdej klatce fizyki**, bez cache i bez zerowania ciągu w oczekiwaniu na wynik.

Granica: jeden mnożnik na część (`thrusterPower` jest per `ModuleRCS`). Bloki z kilkoma dyszami mają mniej stopni
swobody, więc nie każdy statek da się zbalansować do zera. Raport musi to pokazać.

## 3. Model

Dla części `i` i jej dyszy `k` w układzie statku (`vessel.GetTransform()`), z CoM statku jako początkiem:

- `r_ik`: położenie dyszy (nie części, poprawka względem dzisiejszego modelu),
- `d_ik`: kierunek siły (`useZaxis ? −forward : −up`), tylko aktywne transformy (warianty części, jak w `VesselState`),
- `P_i`: `thrusterPower_oryginalny · thrustPercentage / 100`, z poprawką trybu precyzyjnego jak w `VesselState`,
- `s_ik(t, ω)`: przepustnica stock dla komendy, z uwzględnieniem `enablePitch/Yaw/Roll/X/Y/Z`, `fullThrust`,
  `fullThrustMin`. Wzór do potwierdzenia w kodzie stock (etap 1).

Wkład części przy mnożniku 1:

```
f_i = P_i · Σ_k s_ik · d_ik                (siła)
m_i = P_i · Σ_k s_ik · (r_ik × d_ik)       (moment)
F(x) = Σ_i x_i f_i      M(x) = Σ_i x_i m_i
```

Kluczowe założenie, na którym MechJeb już dziś polega: `s_ik` nie zależy od `thrusterPower`. Wtedy `F` i `M` są
liniowe w `x`.

## 4. Funkcja kosztu

Z rzutami prostopadłymi `Π_t = I − t̂ t̂ᵀ` (dla `t = 0`: `Π_t = I`) i `Π_ω` analogicznie:

```
J(x) = w_F · |Π_t F(x)|² / F_ref²       // siła poza kierunkiem translacji (przy samym obrocie: cała siła)
     + w_M · |Π_ω M(x)|² / M_ref²       // moment poza kierunkiem obrotu (przy samej translacji: cały moment)
     + w_x · Σ_i (1 − x_i)²             // trzymaj ciąg wysoko
przy 0 ≤ x_i ≤ 1,  F·t̂ ≥ 0,  M·ω̂ ≥ 0
```

- To zwykły **wypukły QP** w `n` zmiennych (liczba części RCS), bez dodatkowych zmiennych na wielkość siły i momentu.
  Wielkość trzyma człon `w_x`.
- `F_ref`, `M_ref`: największa siła i moment pojedynczej części, żeby wagi były bezwymiarowe.
- **Wagi są asymetryczne celowo.** Błąd kierunku momentu poprawia zamknięta pętla kontrolera orientacji. Siły
  wypadkowej nic nie poprawia, a ona właśnie daje Δv. Domyślnie więc `w_F ≫ w_M`, np. 1 : 0,05, a `w_x` małe
  (10⁻³–10⁻²). Do strojenia w testach.
- Nowa funkcja liczy **prawdziwe siły i momenty**, a nie ważone kierunki dysz jak dziś. Dzisiejsza ścieżka translacji
  zostaje bez zmian, dopóki nowa nie jest domyślna (etap 6).

### Solver

- `alglib.minqp` z ograniczeniami pudełkowymi (jest już w `alglib/optimization.cs`, bez nowej zależności) albo,
  jeśli okaże się za wolny, mały solver z aktywnym zbiorem w `MechJebLib`. Hesjan `n × n` z wektorów `f_i`, `m_i`,
  gradient analityczny, bez numerycznego różniczkowania.
- **Ciepły start** od `x` z poprzedniej klatki i limit iteracji. Komendy kontrolera zmieniają się płynnie, więc
  zwykle wystarczy kilka kroków.
- Budżet: średnio < 0,1 ms na klatkę dla ~24 części. Pomiar wbudowany w okno (jak dzisiejszy „Calculation time”).
- **Awaria = zachowanie stock**, czyli `x = 1`, a nie zerowy ciąg. Licznik awarii widoczny w oknie.

## 5. Integracja w MechJebie

1. **Kolejność.** Balans obrotu musi działać po kontrolerze orientacji. Albo podnieść `Priority` balancera powyżej 800,
   albo rozdzielić go: translacja zostaje na 700, nowy krok rotacji idzie po 800. Najpierw trzeba potwierdzić w grze,
   że `OrderBy` faktycznie daje kolejność rosnącą i że inne moduły nie zależą od obecnej.
2. **Inne źródła komend.** Stock SAS i mody (kOS, inne autopiloty) piszą `FlightCtrlState` w swoich callbackach
   (`OnPreAutopilotUpdate`, `OnAutopilotUpdate`, `OnPostAutopilotUpdate`, `OnFlyByWire`). Trzeba ustalić kolejność
   w KSP 1.12 i udokumentować, które źródła są balansowane. Minimum: komendy MechJeba i klawiatury.
3. **Przełącznik.** Nowa opcja „Smart RCS rotation” w oknie RCS Balancera, osobno od „Smart RCS translation”.
   Domyślnie wyłączona. Zapisywana tak jak dzisiejsze (`Persistent`, `Pass.TYPE | Pass.GLOBAL`).
4. **Estymacja momentu dla kontrolera.** Przy włączonym balansie obrotu `VesselState` powinien:
   - liczyć z **oryginalnego** `thrusterPower`, a nie z bieżącego zmienionego przez balancer,
   - dostać z balancera moment dostępny po balansie w ±pitch/yaw/roll (sześć rozwiązań wzorcowych, przeliczanych
     razem z wykrywaniem przesunięcia CoM).

   Bez tego BetterController przecenia moment i przeregulowuje.
5. **Wykrywanie zmian statku**: dzisiejsze `CheckVessel` (liczba części, włączone moduły, przesunięcie CoM).
   Dochodzą pozycje dysz i zmiana „Control from here”.
6. **Raport w oknie i info items.** Siła wypadkowa i „przeciek” dla bieżącej komendy i dla sześciu komend
   wzorcowych: `(|F|/m) / (|M|/I)` w m/s na rad/s. Do tego procent zachowanego momentu w każdej osi, czas solvera
   i liczba awarii.

## 6. Etapy

Zgodnie z `CONTRIBUTING.md` MechJeba: najpierw issue z projektem, potem małe PR-y, każdy do osobnego przeglądu.

### Etap 0: issue z propozycją (przed kodem)

Opis problemu (sonda bez kół, Principia, Δv z obrotu), streszczenie sekcji 2–5 i pytania do maintainerów:
- czy chcą tego w RCS Balancerze,
- osobny przełącznik czy rozszerzenie istniejącego,
- `alglib.minqp` czy własny solver w `MechJebLib`,
- jak traktować `Priority`.

Dalsze etapy dopiero po zgodzie na kierunek. `CONTRIBUTING.md` wprost mówi, że duże PR bez wcześniejszej dyskusji są
odrzucane. Mówi też, że przy kodzie pisanym z pomocą AI autor PR musi sam rozumieć i umieć bronić decyzji projektowych
oraz być gotowy na utrzymanie kodu.

### Etap 1: rozpoznanie (lokalnie, bez PR)

- Odczytać `ModuleRCS.FixedUpdate` i `ModuleRCSFX` z `Assembly-CSharp.dll` (KSP 1.12.5, ILSpy). Spisać dokładny wzór
  `s_ik(t, ω)`: normalizacja, ramię dyszy czy części, `fullThrust`, `fullThrustMin`, przełączniki osi, tryb precyzyjny,
  `useLever`, Isp zależne od ciśnienia.
- Porównać ze wzorem w `RCSSolver.Thruster.GetThrust`.
- Ustalić kolejność callbacków `FlightCtrlState` w `Vessel` i potwierdzić kolejność `Drive` w MechJebie.
- Sprawdzić moduły RCS z Realism Overhaul / RealFuels (podklasy `ModuleRCS`?).

### Etap 2: model bez zmiany zachowania (PR 1)

- Wyciągnąć model dyszy do testowalnej postaci (położenie i kierunek każdej dyszy, `s_ik` według etapu 1), nadal
  używanej przez obecny solver translacji.
- Testy jednostkowe modelu w `MechJebLibTest`.
- Zachowanie balancera bez zmian. Weryfikacja w grze: translacja jak przed zmianą.

### Etap 3: solver QP (PR 2)

- Funkcja kosztu z sekcji 4 i solver z ciepłym startem, w `MechJebLib` (bez typów Unity), z testami:
  - symetryczny statek: `x = 1`, przeciek ≈ 0,
  - CoM przesunięty: dalszy koniec przyciszony, przeciek po balansie ≈ 0, moment > 0,
  - dysze tylko na jednym końcu: wynik bez wyjątku, niezerowy przeciek zgłoszony,
  - komenda mieszana: `F ∥ t`, `M ∥ ω`,
  - wyłączone osie w częściach respektowane,
  - wydajność: 24 części, sekwencja 1000 płynnie zmiennych komend, średni czas < 0,1 ms,
  - brak zbieżności: wynik `x = 1` i flaga.

### Etap 4: balans obrotu w grze (PR 3)

- Przełącznik „Smart RCS rotation”, wywołanie solvera po kontrolerze orientacji (sekcja 5.1), synchronicznie co
  klatkę, awaria = stock.
- Raport w oknie (sekcja 5.6).
- Translacja nadal przez obecną ścieżkę z wątkiem i cache.

### Etap 5: estymacja momentu (PR 4)

- `VesselState` liczy z oryginalnego `thrusterPower` i bierze moment po balansie z balancera (sekcja 5.4).
- Test w grze: BetterController na sondzie z samym RCS, obrót o 90° i 180°, bez przeregulowania i drgań.

### Etap 6 (opcjonalny): jedna ścieżka dla wszystkiego (PR 5)

Jeśli nowy solver okaże się szybszy i co najmniej tak dobry dla translacji: przenieść na niego także translację
i usunąć wątek oraz cache. Tylko za zgodą maintainerów, bo zmienia zachowanie istniejącej funkcji.

## 7. Testy w grze (etapy 4–5)

1. Sonda bez kół, RCS na obu końcach, CoM wyraźnie przesunięty (pełny zbiornik bliżej jednego końca).
2. Pobliski obiekt jako target, prędkość względna z dokładnością do mm/s.
3. SmartASS: seria obrotów (90°, 180°, roll) z wyłączonym, a potem z włączonym „Smart RCS rotation”. Porównać przyrost
   prędkości względnej z raportem przecieku.
4. Trzymanie orientacji przez kilka minut: liczba impulsów i przyrost prędkości z balansem i bez.
5. Translacja z klawiatury i z MechJeb Docking Autopilot: bez regresji względem „Smart RCS translation”.
6. Komendy mieszane: Docking Autopilot (obrót i translacja naraz).
7. Tryb precyzyjny (Caps Lock), part variants, kilka `ModuleRCS` na jednej części, staging i dokowanie w trakcie.
8. Spalanie paliwa: przeciek po przesunięciu CoM wraca do zera po automatycznym przeliczeniu.
9. Wydajność: czas solvera w oknie przy kilkudziesięciu częściach RCS.

## 8. Ryzyka

- **Wzór stock może być inny niż model MechJeba** (normalizacja, ramię części zamiast dyszy). Etap 1 rozstrzyga.
  Jeśli przepustnica stock zależy od `thrusterPower`, cała liniowość upada i trzeba wrócić do projektu.
- **Kolejność callbacków.** Jeśli stock SAS albo inne mody piszą komendę po MechJebie, ich obroty nie będą
  balansowane. To trzeba udokumentować, nie obchodzić.
- **Akceptacja upstream.** Bez zgody maintainerów zostaje własny fork, który trzeba utrzymywać przy każdej wersji
  MechJeba. Wtedy prostszy wariant w Talarii (`rcs_balans.md`) może być tańszy.
- **Statki nie do zbalansowania.** Raport musi to jasno pokazywać, a nie po cichu dławić ciąg.
- **Wpływ na wcześniej wyliczone ograniczniki.** Balancer mnoży `thrusterPower`, a `thrustPercentage` zostaje.
  Ręczne lub talariowe ograniczniki są więc uwzględnione w `P_i` i nie kolidują.
