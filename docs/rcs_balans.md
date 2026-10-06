# Balans RCS przy obrocie: plan

Plan dodania do Talarii opcji, która dobiera ograniczniki ciągu dysz RCS tak, żeby **obrót sondy nie zmieniał jej
prędkości**. Stan na 4.10.2026, nic z tego nie jest jeszcze zrobione.

## 1. Problem

Sonda bez kół reakcyjnych obraca się samym RCS. Gdy dysze na przodzie i na tyle pchają przy pitch/yaw w przeciwne strony
**z równą siłą**, dostajemy parę sił: czysty moment, zerowa siła wypadkowa, niezależnie od położenia środka masy (CoM).
Stock KSP nie odpala jednak dysz z równą siłą. Ciąg każdej dyszy zależy od jej udziału w żądanym momencie, czyli od
geometrii względem CoM. Gdy CoM nie leży w połowie między pierścieniami dysz, jedna strona pcha mocniej i obrót daje Δv.
CoM przesuwa się przy każdym spalaniu, więc ręcznie ustawione ograniczniki szybko przestają pasować.

Przy Principii to przeszkadza bardziej niż zwykle: każde przypadkowe Δv rozjeżdża plan lotu, a TCM z Hermesa zakłada,
że między korektami statek leci balistycznie.

Istniejące mody tego nie robią:
- MechJeb2 (RCS Balancer, „Smart RCS translation”) balansuje tylko translację, żeby przesuwanie nie obracało statku.
- Throttle Controlled Avionics zmienia ograniczniki RCS w locie, ale wprost obiecuje tylko usuwanie momentu przy
  translacji.
- RCS Build Aid tylko pokazuje problem w edytorze.
- Plan pełnego balansu obrotu w samym MechJebie (rozszerzenie RCS Balancera) jest w `mechjeb_rcs_rotacja.md`.
- PWB Fuel Balancer przywraca CoM przepompowywaniem paliwa. Wymaga zbiorników po obu stronach CoM.

## 2. Pomysł

Ograniczniki ciągu dysz (`ModuleRCS.thrustPercentage`, w PPM „RCS Thrust Limiter”) skalują ciąg liniowo. Dla ustalonej
komendy obrotu siła wypadkowa i moment są więc **liniowe względem ograniczników**. Dobór ograniczników to mały problem
najmniejszych kwadratów z ograniczeniami 0–100%, a nie zgadywanie.

Funkcja **nie używa Principii**. Czyta tylko części statku i ustawia pole stock KSP, więc nie grożą jej crashe
z `CHECK` Principii, a przy nieprzetestowanej wersji Principii (tryb tylko do odczytu) może działać normalnie.

### Model

Dla każdej dyszy `k` (każdy `thrusterTransform` w części z `ModuleRCS`):
- `r_k`: położenie względem CoM statku,
- `d_k`: kierunek siły, czyli przeciwny do wylotu,
- `T_k`: maksymalny ciąg w aktualnych warunkach (w próżni `thrusterPower`),
- `t_k(u)`: przepustnica, jaką stock nada dyszy dla komendy `u` (pitch, yaw, roll, translacja), w przedziale 0–1,
- `l_j`: ogranicznik części `j`, do której należy dysza (jeden na część, wspólny dla wszystkich jej dysz).

Dla komendy `u`:

```
F(u) = Σ_k  l_j(k) · T_k · t_k(u) · d_k
τ(u) = Σ_k  l_j(k) · T_k · t_k(u) · (r_k × d_k)
```

Kluczowe założenie do sprawdzenia: `t_k(u)` **nie zależy od ograniczników**. Wtedy `F` i `τ` są liniowe w `l`.

### Co optymalizujemy

Sześć komend obrotu: ±pitch, ±yaw, ±roll (każda osobno, z pełnym wychyleniem). Szukamy `l ∈ [l_min, 1]^J`, które
minimalizuje

```
Σ_u |F(u)|² / F_ref²  +  μ · Σ_j (1 − l_j)²
```

- Pierwszy człon zeruje siłę wypadkową przy obrocie.
- Drugi trzyma ograniczniki jak najwyżej, żeby nie stracić momentu sterującego. Bez niego rozwiązaniem byłoby
  `l = 0`.
- `F_ref` to typowa siła jednej dyszy, żeby człony miały podobną skalę. `μ` to mała waga (np. 10⁻³), do dostrojenia
  w testach.
- `l_min` (np. 10%) nie pozwala wyłączyć części całkiem.

Opcjonalnie dochodzi drugi zestaw komend: sześć komend translacji z członem `|τ(u)|²`, żeby nie zepsuć balansu
translacji, z którego korzysta np. MechJeb przy dokowaniu. Domyślnie z mniejszą wagą, bo celem jest obrót.

Funkcja celu jest wypukła i kwadratowa, z ograniczeniami pudełkowymi. Wystarczy prosty rzutowany spadek gradientu albo
coordinate descent. Części RCS jest zwykle kilka do kilkunastu, więc to ułamek milisekundy.

### Miara przecieku

Dla raportu w oknie potrzebna jest liczba, którą da się porównać z budżetem Δv. Przy obrocie z pełnym wychyleniem
prędkość kątowa rośnie o `|τ|/I`, a prędkość liniowa o `|F|/m`. Stosunek

```
przeciek(u) = (|F(u)| / m) / (|τ(u)| / I)     [m/s na rad/s]
```

mówi, ile Δv kosztuje rozpędzenie sondy do danej prędkości kątowej (`I` to moment bezwładności wokół osi komendy).
Przykład: 0,05 m/s na rad/s i obrót z prędkością 0,75°/s (0,013 rad/s) dają ok. 0,65 mm/s na rozpędzenie i tyle samo
na hamowanie. Raport pokazuje przeciek przed balansem i po nim, dla każdej osi.

## 3. Ryzyka i pytania otwarte

1. **Dokładny algorytm stock.** Nie wiemy na pewno, jak `ModuleRCS` liczy przepustnicę dyszy przy obrocie (czy używa
   ramienia znormalizowanego czy pełnego, jaki ma próg, co robi „Always full action”, tryb precyzyjny, przełączniki osi
   Pitch/Yaw/Roll w PPM). Od tego zależy model `t_k(u)`. Etap A to rozstrzyga.
2. **`ModuleRCSFX` i Realism Overhaul.** Przy RSS części RCS mogą używać `ModuleRCSFX` (podklasa stock) albo modułu
   z Realism Overhaul / RealFuels, z inną logiką ciągu lub z Isp zależnym od ciśnienia. Model musi obsłużyć klasę, której
   faktycznie używają części gracza.
3. **Konflikt z innymi modami, które piszą `thrustPercentage`.** MechJeb (Smart RCS translation) i TCA mogą nadpisywać
   ograniczniki co klatkę. Talaria wykrywa je i ostrzega, nie walczy z nimi.
4. **Nie każdy statek da się zbalansować.** Dysze tylko na jednym końcu albo części z wieloma dyszami (jeden
   ogranicznik na 4 dysze) mogą nie dać zera. Raport pokazuje przeciek po balansie, a gdy moment w którejś osi spadnie
   poniżej np. 30% pierwotnego, ostrzega.
5. **Kolejność aktualizacji.** Zmiana ograniczników w trakcie odpalania dysz daje szarpnięcie. Tryb statyczny
   (etapy C–D) zmienia je tylko wtedy, gdy RCS nie dostaje żadnej komendy.
6. **Układ odniesienia komend.** Pitch/yaw/roll są względem `vessel.ReferenceTransform` (część „Control from here”),
   ze znakami do sprawdzenia w etapie A.

## 4. Etapy

Każdy etap to osobny PR w repozytorium Talarii, ułożone w stos zgodnie z jej `CLAUDE.md`.

### Etap A: rozpoznanie (lokalnie, bez PR z kodem)

- Odczytać `ModuleRCS.FixedUpdate` (i `ModuleRCSFX`) z `Assembly-CSharp.dll` z KSP_DEV, np. ILSpy. Zapisać w tym pliku
  dokładny wzór na przepustnicę dyszy przy obrocie i translacji, rolę `thrustPercentage`, `fullThrust`,
  `enablePitch/Yaw/Roll/X/Y/Z`, `useZaxis`, trybu precyzyjnego i Isp zależnego od ciśnienia.
- Sprawdzić, jakich modułów RCS używają części w instalacji gracza (RO/RealFuels).
- Sprawdzić w kodzie MechJeb2 (RCS Balancer) i TCA, czy i kiedy zapisują `thrustPercentage`.
- Rozstrzygnąć założenie, że przepustnica nie zależy od ogranicznika.

Wynik: uzupełniona sekcja „Model” w tym pliku.

### Etap B: model i solver (wersja 0.11.1)

Nowy plik `src/Talaria/RcsBalance.cs`, bez typów Unity/KSP, dodany do `Talaria.Tests.csproj`:
- wejście: lista dysz (`Vec3` położenia i kierunku, ciąg, indeks części, włączone osie), masa, moment bezwładności,
- `Throttle(nozzle, command)`: wzór stock z etapu A,
- `Solve(...)`: ograniczniki części według funkcji celu z sekcji 2,
- `Report(...)`: siła, moment i przeciek dla każdej z sześciu komend, przed i po.

Testy jednostkowe:
- symetryczny statek z CoM w środku: wszystkie ograniczniki zostają na 100%, przeciek ~0,
- CoM przesunięty w stronę jednego końca: ograniczniki dalszego końca spadają, przeciek po balansie bliski zera,
- dysze tylko na jednym końcu: solver kończy się bez wyjątku, raport pokazuje niezerowy przeciek i ostrzeżenie,
- wyłączona oś w PPM dyszy jest uwzględniona,
- ograniczenie `l_min` jest przestrzegane.

### Etap C: przycisk „Balance RCS” (wersja 0.12.0)

W oknie Talarii nowa sekcja „RCS balance”:
- **Balance RCS now**: zbiera dysze z części aktywnego statku (`ModuleRCS` i podklasy), liczy CoM (`vessel.CoM`)
  i moment bezwładności, uruchamia solver i ustawia `thrustPercentage`. Tylko gdy RCS nie odpala (brak komend obrotu
  i translacji).
- **Reset to 100%**: przywraca pełne ograniczniki.
- Raport: przeciek w każdej osi przed/po (mm/s na °/s), procent zachowanego momentu, ostrzeżenia (przeciek nie do
  usunięcia, słaba oś, wykryty MechJeb Smart RCS translation lub TCA).
- Wpis do `KSP.log` z prefiksem `[Talaria]`.

Ograniczniki są polem zapisywanym w save'ie KSP, więc przetrwają wczytanie gry bez dodatkowego zapisu po stronie moda.

### Etap D: automatyczny rebalans (wersja 0.12.1)

- Przełącznik **Auto** (zapisywany w `settings.cfg`): co sekundę porównuje CoM w układzie części root z położeniem przy
  ostatnim balansie. Gdy przesunięcie przekroczy próg (np. 1% odległości między pierścieniami dysz) i RCS nie odpala,
  liczy balans od nowa.
- Rebalans także po `GameEvents.onVesselWasModified` (staging, dokowanie) i po zmianie „Control from here”.
- Brak rebalansu w time warpie na szynach i gdy statek nie jest aktywny.

### Etap E (opcjonalny): balans dynamiczny dla bieżącej komendy

Zamiast sześciu komend wzorcowych liczyć ograniczniki co klatkę fizyki dla **aktualnej** komendy (mieszanki obrotu
i translacji, także z MechJeba). To dokładniejsze przy złożonych manewrach, ale wymaga ustawienia ograniczników
po autopilotach, a przed `ModuleRCS.FixedUpdate` (np. `vessel.OnPostAutopilotUpdate`, do sprawdzenia) i koliduje
z MechJebem i TCA. Decyzja dopiero po doświadczeniach z etapami C–D.

## 5. Test w grze (po etapie C)

1. Sonda z RCS na obu końcach i CoM wyraźnie przesuniętym do jednego z nich (np. pełny zbiornik bliżej przodu).
2. Ustaw pobliski obiekt jako target i obserwuj prędkość względną z dokładnością do mm/s.
3. Bez balansu wykonaj kilka obrotów pitch/yaw tam i z powrotem i zanotuj przyrost prędkości względnej.
4. Kliknij **Balance RCS now**, porównaj raport z PPM dysz i powtórz obroty. Przyrost powinien być wyraźnie mniejszy,
   a raport powinien zgadzać się z pomiarem co do rzędu wielkości.
5. Spal część paliwa, sprawdź, że raport pokazuje wzrost przecieku, i powtórz balans (w etapie D: że Auto zrobiło to
   samo).
6. Sprawdź translację (H/N/I/J/K/L) po balansie: czy nie zaczęła obracać sondy bardziej niż przed balansem.
7. Przy problemach odeślij `KSP.log` i `Player.log`.
