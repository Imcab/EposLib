# HANDOFF — Driver EPOS4 / CANopen / ros2_control

> **Documento de traspaso entre sesiones.**
> Escrito el **2026-09-24**, al cerrar la sesión en la que se terminó la
> **Parte B** (el plugin de `ros2_control`).
> Workspace: `/data/epos_controller_ws` · rama `main` · último commit `b411991`.

---

## 0. Cómo usar este documento

Este archivo existe para que la siguiente sesión arranque **sin volver a
descubrir nada**. Contiene el encargo, las reglas de trabajo, el estado
verificado del código, los conceptos que ya se explicaron (para no
re-explicarlos), las trampas que ya se pisaron, y los siguientes pasos con
criterios de aceptación concretos.

### Prompt sugerido para abrir la siguiente sesión

```
Lee /data/epos_controller_ws/HANDOFF.md completo antes de hacer nada.
Ahí está todo: el encargo, las reglas de trabajo, el estado del código,
lo que falta y cómo verificarlo.

Retomamos en la sección «11. Siguientes pasos», punto 1.
```

### Índice

| § | Sección |
|---|---|
| 1 | El encargo y quién lo pide |
| 2 | Reglas de trabajo (**leer sí o sí**) |
| 3 | Entorno verificado |
| 4 | Mapa del workspace |
| 5 | Arquitectura y las dos fronteras |
| 6 | Recorrido paquete por paquete |
| 7 | Conceptos ya cubiertos (no re-explicar) |
| 8 | Parte A — el camino de tiempo real |
| 9 | Parte B — el plugin de ros2_control |
| 10 | Parte C — el simulador y el bloqueo del remapeo PDO |
| 11 | Siguientes pasos, en orden |
| 12 | Plantillas de los archivos que faltan |
| 13 | Receta de verificación end-to-end |
| 14 | Estado de build y tests (números reales) |
| 15 | Qué se arregló en esta sesión |
| 16 | Trampas ya pisadas (tabla larga) |
| 17 | Mensajes de commit sugeridos |
| 18 | Deuda técnica y backlog |
| 19 | Referencias del manual |
| 20 | Glosario |

---

## 1. El encargo y quién lo pide

**Imad** está construyendo el **brazo robótico del rover del equipo Quantum**
para la competencia **URC** (University Rover Challenge). Stack: **ROS 2
Humble** sobre Ubuntu 22.04.

Existía una librería previa, escrita por un compañero **en Python**, en
`/data/quantumrepositories/epos_control`. Funcionaba, pero estaba moldeada
alrededor de *ese* brazo: los node-IDs, los nombres de las articulaciones y los
parámetros de movimiento estaban incrustados en el código. Era un parche útil,
no una librería.

**El encargo es refactorizar eso en una librería maxon de verdad, en C++, sobre
Lely CANopen.**

### El listón de calidad, en palabras de Imad

> *"acuérdate que estamos haciendo UNA api, no 'algo' específico para nuestro
> brazo... me gustaría que fuera como la api de CTRE... que tenga todos los
> métodos, que sea configurable"*

La referencia explícita es **CTRE Phoenix 6**:
<https://api.ctr-electronics.com/phoenix6/stable/java/>

Eso se traduce en decisiones concretas que **ya están tomadas y aplicadas**, y
que hay que respetar al seguir:

- **Objetos-dispositivo, no funciones sueltas.** `Epos4 motor{bus, 2};`
- **Configuración por grupos declarativos**, todos los campos `std::optional`,
  aplicados con un solo `Apply()`. Como los `*Configs` de Phoenix.
- **Status signals** con caché, marca de tiempo y antigüedad
  (`GetPosition().Refresh().GetValue()`). Como los `StatusSignal<T>` de Phoenix.
- **Control requests** encadenables:
  `SetControl(ProfilePosition{}.WithPosition(90_deg).WithVelocity(2000))`.
  Como los `ControlRequest` de Phoenix.
- **Subsistemas como objetos propios**: `motor.GetEncoder()`,
  `motor.GetConfigurator()`.
- **Unidades tipadas** con análisis dimensional en tiempo de compilación.

### Segundo encargo, derivado

El paquete `robot_units` **no es para el EPOS4**. Imad lo pidió aparte, genérico,
para reutilizarlo en todo el robot:

> *"me gustaría que incluyeras un paquete de 'Unidades' que sea como en wpi...
> que sea genérica porque sí lo quiero usar en mi robot"*

Y después, tras rechazar dos intentos propios:

> *"olvídalo, ya vi cómo lo hace wpi en c++ y mira, básate 100% EN ESTA
> https://github.com/nholthaus/units.git"*

Resultado: `robot_units` **vendoriza nholthaus/units v2.3.5** tal cual. No se
escribe una librería de unidades propia. Punto cerrado.

---

## 2. Reglas de trabajo — LEER SÍ O SÍ

Estas reglas vienen de instrucciones explícitas de Imad a lo largo de la
sesión. Romperlas es el fallo más caro que puede cometer la siguiente sesión.

### 2.1 Git: NO ejecutar comandos de git que modifiquen nada

> *"yo quiero darle commits, push, todo, tú solo dame la descripción del commit"*

**Regla absoluta:**

- ❌ NO `git add`, NO `git commit`, NO `git push`, NO `git checkout -b`,
  NO `git stash`, NO `git reset`.
- ✅ SÍ `git status`, `git diff`, `git log` (solo lectura, para informar).
- ✅ SÍ **redactar el texto del mensaje de commit** y dárselo a Imad para que
  él lo pegue.

En §17 hay mensajes de commit ya redactados para el trabajo pendiente de
commitear.

### 2.2 El modo de trabajo cambió a mitad de proyecto

**Al principio** Imad pidió tutoría estricta:

> *"No quiero que escribas código por mí — yo lo voy a escribir. Quiero que
> actúes como tutor."*

**Eso quedó superseded.** A mitad de proyecto:

> *"mira, cambio de planes, necesitamos acelerar esto porque lo necesitamos
> cuanto antes, mi líder de área me dijo que no hay problema y que sigamos la
> filosofía de hacerlo- luego entenderlo, ok? así que podrías ir haciendo paso
> a paso todo pero justamente explicándome? o sea ir escribiendo bloques y
> bloques"*

Y más adelante, aún más claro:

> *"mira sabes qué, podrías hacerlo tú? es que esto son cosas que como que nos
> están frenando para avanzar"*

**Modo actual: escribir el código completo, explicando mientras se escribe.**
No preguntar "¿qué has intentado?" antes de dar código. Esa fase terminó.

### 2.3 Escribir SIEMPRE en `src/`, nunca en el scratchpad

Hubo un incidente:

> *"oye qué has escrito? porque no sé qué has escrito y no veo"*

Se había trabajado en un directorio temporal y Imad no veía los archivos en su
editor. **Todo el código va directo a `/data/epos_controller_ws/src/`.** El
scratchpad solo para salidas intermedias desechables.

### 2.4 Idioma

> *"estandaricemos todo a inglés, todos los comentarios, nombres de los test,
> etc"*

- **Código, comentarios, nombres de test, mensajes de log, README, CHANGELOG:
  INGLÉS.**
- **Conversación con Imad y documentos de traspaso como este: ESPAÑOL.**

### 2.5 Estilo de comentarios

El código existente tiene una voz muy marcada y hay que mantenerla. Los
comentarios **no dicen qué hace el código, dicen por qué es así**, y casi
siempre citan el manual o explican un fallo que evita. Ejemplos reales del
repo:

```cpp
// Devices BEFORE Start(). A CANopen master boots each slave once, at
// reset, and only routes a node's PDOs to a driver registered at that
// moment. Constructing them afterwards gives nodes that answer SDO
// perfectly and never deliver a single PDO - which here would look like
// every joint frozen at its startup position.
```

```cpp
// Seed the command with where the axis already is. Without this the first
// write() sends the default command of zero and the arm swings to its
// origin the instant the controller activates.
```

```cpp
// NOTE: what Halt does is formally decided by «Halt option code» (0x605D,
// Table 6-152). That object is documented in the manual but is NOT present
// in the EDS of the EPOS4 Module 50/15, so it is not exposed here - adding
// a setter for an object the device does not implement would only produce
// aborts at run time.
```

Nótese: comillas angulares «» para nombres de estados y objetos del manual,
referencias a tablas y secciones, y el patrón "sin esto, pasa X".

### 2.6 Regla de oro del diccionario de objetos

**Ningún archivo fuera de `ObjectDictionary.hpp` contiene una dirección
hexadecimal de objeto desnuda.** Todo va por constante:
`od::cia402::kControlword`, `od::maxon::kMotorData`. Esto ya se cumple en todo
el repo; hay que seguir cumpliéndolo.

### 2.7 Auditorías cuando se pidan

Imad pidió explícitamente una auditoría de los enums contra el manual:

> *"revisa detalladamente si en algún enum no escribiste mal los datos como el
> homing method"*

Se hizo y **aparecieron tres errores reales** (ver §16). Si pide una auditoría,
hacerla en serio, tabla por tabla contra el PDF.

---

## 3. Entorno verificado

Todo lo de esta tabla se comprobó ejecutándolo, no de memoria.

| Cosa | Estado | Comando de verificación |
|---|---|---|
| Workspace | `/data/epos_controller_ws` | — |
| ROS | **Humble** (`/opt/ros/humble`) | `echo $ROS_DISTRO` |
| Lely | **`ros-humble-lely-core-libraries 0.2.13-1jammy`** instalado | `dpkg -l \| grep lely` |
| `ros2_control` | instalado | `ros2 pkg list \| grep ros2_control` |
| `vcan0` | ❌ **NO existe ahora mismo** | `ip link show vcan0` |
| CAN físico | Sin `can0`. Imad **sí tiene acceso al EPOS4**, pero no de forma continua | — |
| Compilador | GCC, C++17 | — |
| Manuales | En `resources/`, **gitignored** | `ls resources/` |

### Levantar `vcan0` (hace falta cada reinicio)

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
ip link show vcan0      # debe aparecer <NOARP,UP,LOWER_UP>
```

### Los manuales (no están en git, ~14 MB)

```
resources/EPOS4-Firmware-Specification-En-1.pdf    310 pág., ed. 2026-07, rel13740
resources/EPOS4-Communication-Guide-En.pdf         ed. 2026-04, rel13604
```

Descargables gratis de maxongroup.com. **Dato muy útil: en el PDF de firmware
la página del visor coincide 1:1 con la página impresa**, así que:

```bash
pdftotext -f 215 -l 216 -layout resources/EPOS4-Firmware-Specification-En-1.pdf -
```

salta directo a Controlword/Statusword.

### El EDS

`src/epos4_bringup/config/epos4_network/epos4.eds` (5754 líneas) es el EDS real
exportado de **EPOS Studio**, modelo **EPOS4 Module 50/15**.

> ⚠️ **Discrepancia sin confirmar.** En una captura que compartió Imad, la
> página que estaba navegando mostraba un **EPOS4 Compact 50/5**. El EDS del
> repo es del **Module 50/15**. Los objetos CiA 402 son los mismos, pero los
> límites de corriente y los datos de identidad no. **Hay que confirmar el
> modelo físico antes de conectar hardware.**

---

## 4. Mapa del workspace

```
/data/epos_controller_ws/
├── .gitignore                    build/, install/, log/, *.dcf, *.bin, resources/*.pdf
├── README.md                     ~15 KB, documentación de usuario (SIN COMMITEAR)
├── HANDOFF.md                    este archivo
├── resources/                    los 2 PDFs de maxon (gitignored)
├── build/ install/ log/          artefactos de colcon (gitignored)
└── src/
    ├── robot_units/              nholthaus/units vendorizado
    ├── epos4_driver/             LA LIBRERÍA
    ├── epos4_interfaces/         vacío (solo el esqueleto del paquete)
    ├── epos4_ros2_control/       EL PLUGIN  ← terminado en esta sesión
    └── epos4_bringup/            EDS, bus.yml, DCFs, launch (launch VACÍO)
```

### Inventario completo con conteo de líneas

```
   LÍNEAS  ARCHIVO
──────────────────────────────────────────────────────────────────────
           robot_units/
       35  robot_units/CMakeLists.txt
       28  robot_units/include/robot_units.hpp
       21  robot_units/LICENSE-units.txt
       25  robot_units/package.xml
      104  robot_units/test/test_units.cpp
           (+ include/units.h vendorizado, ~6000 líneas, no listado)

           epos4_driver/  — cabeceras
       89  include/epos4/CanBus.hpp
      443  include/epos4/configs/Configs.hpp
      247  include/epos4/configs/EncoderConfigs.hpp
      160  include/epos4/controls/ControlRequests.hpp
       48  include/epos4/core/BusImpl.hpp
      199  include/epos4/core/Cia402StateMachine.hpp
      688  include/epos4/core/ObjectDictionary.hpp
      134  include/epos4/core/Setpoint.hpp
      163  include/epos4/core/UnitConversion.hpp
      180  include/epos4/hardware/Encoder.hpp
      436  include/epos4/hardware/Epos4.hpp
      408  include/epos4/signals/Enums.hpp
      138  include/epos4/signals/Errors.hpp
       77  include/epos4/signals/StatusSignal.hpp
       55  include/epos4/Version.hpp

           epos4_driver/  — implementación
      201  src/CanBus.cpp
      383  src/configs/Configs.cpp
      231  src/configs/EncoderConfigs.cpp
      200  src/core/Cia402StateMachine.cpp
      223  src/hardware/Encoder.cpp
     1434  src/hardware/Epos4.cpp
      279  src/signals/Enums.cpp
     1248  src/signals/Errors.cpp
       18  src/Version.cpp

           epos4_driver/  — tests (95 casos gtest)
      313  test/test_cia402_state_machine.cpp      18 tests
      345  test/test_configs.cpp                   18 tests
      248  test/test_digital_inputs.cpp            14 tests
      194  test/test_encoder_configs.cpp           13 tests
      172  test/test_errors.cpp                    14 tests
      253  test/test_unit_conversion.cpp           18 tests

           epos4_driver/  — herramientas y ejemplos
      493  tools/epos4_sim.cpp                     el simulador
       53  examples/01_event_loop.cpp
       80  examples/02_can_listen.cpp
      179  examples/03_master_boot.cpp
      155  examples/api_demo.cpp
      141  CMakeLists.txt
       31  package.xml
      115  CHANGELOG.md

           epos4_ros2_control/  — EL PLUGIN
      101  include/epos4_ros2_control/Epos4System.hpp
      362  src/Epos4System.cpp
       11  epos4_ros2_control_plugin.xml
      168  test/test_epos4_system.cpp              12 tests
       76  CMakeLists.txt
       36  package.xml

           epos4_bringup/
       33  CMakeLists.txt
       29  package.xml
       79  config/epos4_network/bus.yml
     5754  config/epos4_network/epos4.eds
       89  config/sim_network/bus.yml
     1442  config/sim_network/sim_slave.eds
        —  launch/                                 ⚠️ DIRECTORIO VACÍO

           epos4_interfaces/
       37  CMakeLists.txt
       29  package.xml
        —  msg/ srv/                               ⚠️ NO EXISTEN
```

### ⚠️ Basura que limpiar

Existe `src/epos4_driver/install/` — **2.6 MB de artefactos de colcon dentro
del árbol de fuentes.** Alguien ejecutó `colcon build` desde dentro del
paquete. Está cubierto por `.gitignore` (`**/install/`) así que **no se va a
commitear**, pero confunde a `grep` y a los linters. Borrar:

```bash
rm -rf src/epos4_driver/install src/epos4_driver/build src/epos4_driver/log
```

---

## 5. Arquitectura y las dos fronteras

El refactor entero se sostiene sobre **dos fronteras**, y ambas están
impuestas por el sistema de build, no por buena voluntad:

```
┌──────────────────────────────────────────────────────────────────┐
│  epos4_core        lógica CiA 402 pura.                          │
│                    NO conoce Lely. NO conoce CAN. NO conoce ROS. │
│                    → Cia402StateMachine, ObjectDictionary,       │
│                      UnitConversion, Setpoint, Enums, Errors     │
│                    → testeable entero sin bus                    │
└──────────────────────────────────────────────────────────────────┘
                              ▲
                              │ (epos4 enlaza contra epos4_core)
┌──────────────────────────────────────────────────────────────────┐
│  epos4             el dispositivo. Usa Lely.                     │
│                    NO conoce ROS.                                │
│                    → CanBus, Epos4, Encoder, Configurator        │
└──────────────────────────────────────────────────────────────────┘
                              ▲
                              │ (el plugin enlaza contra epos4)
┌──────────────────────────────────────────────────────────────────┐
│  epos4_ros2_control   el puente. Aquí y solo aquí entra ROS.     │
│                    → Epos4System : SystemInterface               │
└──────────────────────────────────────────────────────────────────┘
```

**Por qué importa:** si mañana el equipo quiere usar el driver desde un test de
consola, desde otro framework o sin ROS, compila igual. Eso es lo que lo
convierte en librería y no en un nodo con funciones sueltas. Es la diferencia
estructural más importante frente a la versión Python.

`epos4_driver` genera **dos targets de CMake**: `epos4_core` (sin Lely) y
`epos4` (con Lely). La frontera es literal: si alguien mete un `#include` de
Lely en `epos4_core`, el enlace falla.

**`epos4_driver` no incluye ni un solo header de ROS.** Verificable:

```bash
grep -rn "rclcpp\|ros/" src/epos4_driver/include src/epos4_driver/src
# debe salir vacío
```

---

## 6. Recorrido paquete por paquete

### 6.1 `robot_units`

**Qué es:** nholthaus/units v2.3.5 vendorizado. Un solo header, C++14,
análisis dimensional en tiempo de compilación, cero coste en runtime.

**Por qué vendorizado y no como dependencia:** no está empaquetado para ROS
Humble, y el equipo quiere el mismo header disponible para todo el robot sin
depender de un apt que no existe.

```cpp
#include <robot_units.hpp>

using namespace units::literals;
units::angle::turn_t     giro  = 90_deg;    // convierte solo
units::angle::radian_t   rad   = giro;      // convierte solo
// units::angle::degree_t x = 5_A;          // ERROR DE COMPILACIÓN
```

**Trampa documentada en los tests:** `std::is_convertible<ampere_t, degree_t>`
devuelve **`true`**. nholthaus valida con `static_assert` *dentro del
constructor*, no con SFINAE, así que el trait miente. Hay un test que lo deja
por escrito para que nadie construya SFINAE encima de ese trait.

8 tests. `test/test_units.cpp`.

---

### 6.2 `epos4_driver` — la librería

#### `include/epos4/core/ObjectDictionary.hpp` (688 líneas)

**556 constantes** de índice/subíndice, generadas a partir del EDS real del
dispositivo. Divididas en cuatro namespaces:

| namespace | qué contiene | portable |
|---|---|---|
| `od::comm` | CiA 301: `0x1000`–`0x1FFF` (identidad, heartbeat, SDO, PDO, error history) | a cualquier nodo CANopen |
| `od::maxon_comm` | Comunicación específica de maxon: `0x2000`, `0x2001` (node-ID, bit rate) | no |
| `od::maxon` | Todo lo propio de maxon: `0x3000`+ (sensores, motor data, entradas/salidas digitales, current demand) | no |
| `od::cia402` | El perfil de drive: `0x6000`–`0x60FF` (controlword, statusword, modos, posiciones, homing) | **a cualquier drive CiA 402** |

El tipo `od::Entry` empaqueta índice + subíndice, y es lo que consumen
`ReadObject`/`WriteObject`.

**La regla:** ninguna dirección hexadecimal desnuda fuera de este archivo.

#### `include/epos4/core/Cia402StateMachine.hpp` + `src/core/Cia402StateMachine.cpp`

Lógica pura, sin CAN. Tres piezas:

```cpp
// Statusword crudo → uno de los 8 estados de la Tabla 2-5.
// Enmascara con 0x6F los bits que el manual marca «don't care».
// nullopt si el patrón no coincide con ninguno, en vez de inventar un estado.
std::optional<signals::State> Decode(std::uint16_t statusword);

// Construye los comandos de la Tabla 2-7 como read-modify-write:
// preserva los bits de modo de operación y genera el FLANCO que
// «Fault reset» (bit 7) requiere — no es por nivel.
namespace Controlword { ... }

// «¿Qué mando este ciclo?», dado dónde está el drive y a dónde quiero ir.
// SIN ESTADO: el drive reporta dónde está cada ciclo, así que no hay
// progreso que rastrear. Nunca emite un fault reset por su cuenta.
Step PlanStep(signals::State current, Goal goal);
```

18 tests, ninguno necesita bus.

**Los 8 estados** (Tabla 2-5): `NotReadyToSwitchOn`, `SwitchOnDisabled`,
`ReadyToSwitchOn`, `SwitchedOn`, `OperationEnabled`, `QuickStopActive`,
`FaultReactionActive`, `Fault`.

#### `include/epos4/core/UnitConversion.hpp` (163 líneas)

```cpp
class MechanismScale
{
  MechanismScale(std::uint32_t quadCountsPerRevolution, double gearRatio = 1.0);

  units::angle::turn_t ToAngle(std::int32_t quadCounts) const;
  std::int32_t         ToQuadCounts(units::angle::turn_t angle) const;
  units::angular_velocity::revolutions_per_minute_t ToAngularVelocity(std::int32_t rpm) const;
  std::int32_t         ToRpm(units::angular_velocity::revolutions_per_minute_t v) const;
  double               ToRpmPerSecondAtOutput(std::uint32_t rpmPerSecond) const;
  std::uint32_t        FromRpmPerSecondAtOutput(double atOutput) const;
};

// Libres, porque no dependen del mecanismo:
units::torque::newton_meter_t ToTorque(std::int16_t perThousand, std::uint32_t ratedTorqueMicroNm);
std::int16_t                  ToPerThousand(units::torque::newton_meter_t t, std::uint32_t rated);
units::current::ampere_t      ToCurrent(std::int32_t milliamps);
std::int32_t                  ToMilliamps(units::current::ampere_t current);
```

**Detalle importante: redondea al más cercano, no trunca.** Truncar perdía una
cuenta en cada conversión y había un test de 10 pasadas ida-y-vuelta que lo
detectó (`60_rpm` se convertía en `5999`). El helper `Round()` es privado y
maneja negativos.

18 tests.

#### `include/epos4/core/Setpoint.hpp` (134 líneas)

`Setpoint<Raw, Quantity>` acepta **las dos formas**: el valor crudo que usan el
manual y EPOS Studio, o una cantidad con unidades.

```cpp
motor.SetControl(controls::ProfilePosition{}.WithPosition(50000));    // quadcounts
motor.SetControl(controls::ProfilePosition{}.WithPosition(90_deg));   // ángulo
```

**Trampa resuelta:** `degree_t → turn_t → Setpoint` son *dos* conversiones
definidas por el usuario, y C++ solo permite una. Solución: el constructor está
templatizado sobre la unidad de entrada con `units::traits::is_convertible_unit_t`.

**Segunda trampa:** los dos constructores SFINAE colisionaban. Los parámetros
de plantilla de **tipo** con valor por defecto se ignoran al comparar firmas;
hubo que distinguirlos con parámetros **no-tipo** de tipos distintos
(`int` vs `bool`).

#### `include/epos4/signals/Enums.hpp` + `src/signals/Enums.cpp`

Todos los enums del perfil: `State`, `OperationMode`, `HomingMethod`,
`QuickStopOption`, `FaultReactionOption`, `BrakeState`, `SensorType`,
`status_bits::*`…

**Auditado contra el manual a petición de Imad.** Se encontraron 3 errores
reales, ver §16.

#### `include/epos4/signals/Errors.hpp` + `src/signals/Errors.cpp` (1248 líneas)

- **Los 73 device errors de la Tabla 7-186**, cada uno con causa, efecto y
  recuperación transcritos de las secciones 7.2.1 a 7.2.72.
- **Los 26 abort codes SDO de la Tabla 7-187.**
- Cubre los **tres rangos** (`0x1080–0x1088`, `0x5480–0x5483`,
  `0x6180–0x61F0`) que una búsqueda puntual se saltaría.
- `IsWarning()` distingue los códigos con los que el drive sigue funcionando.
- `ClearsPosition()` marca los seis errores cuyo reset pierde la referencia de
  homing — crítico para un brazo con encoders incrementales.

```cpp
std::string DescribeDeviceError(std::uint16_t code);
// → "0x8611 Following error — Cause: the position controller could not
//    follow the setpoint... Effect: ... Recovery: ..."
```

14 tests.

#### `include/epos4/signals/StatusSignal.hpp` (77 líneas)

El equivalente del `StatusSignal<T>` de Phoenix: valor cacheado, marca de
tiempo, antigüedad, y `Refresh()` explícito.

```cpp
auto & pos = motor.GetPosition();
pos.Refresh();                    // fuerza una lectura (SDO o del caché PDO)
std::int32_t counts = pos.GetValue();
auto edad = pos.GetTimeSinceUpdate();
```

**18 señales de estado** expuestas por `Epos4`: `State`, `Statusword`,
`OperationMode`, `Position`, `PositionDemand`, `Velocity`, `VelocityDemand`,
`Torque`, `FollowingError`, `CurrentDemand`, `MotorRatedTorque`, `ErrorCode`,
`ErrorRegister`, `DigitalInputs`, `DigitalInputPins`, `BrakeState`, y las de
bits del Statusword específicos de modo.

#### `include/epos4/configs/Configs.hpp` (443 líneas) + `src/configs/Configs.cpp`

**17 grupos de configuración:**

```
MotorConfigs            GearConfigs             AxisConfigs
CurrentControlConfigs   PositionControlConfigs  VelocityControlConfigs
MotionProfileConfigs    CyclicConfigs           SiUnitConfigs
LimitConfigs            HomingConfigs           HoldingBrakeConfigs
StandstillConfigs       DigitalInputConfigs     DigitalOutputConfigs
StopOptionConfigs       Epos4Configuration  (el agregado de todos)
```

**Cada campo es `std::optional`.** Solo se escribe lo que se fijó
explícitamente, así que una configuración parcial **no puede borrar ganancias
ya ajustadas**. Eso es deliberado y es una de las mejores decisiones del
diseño.

```cpp
epos4::configs::Epos4Configuration cfg;
cfg.positionControl.p             = 1500000;
cfg.motionProfile.profileVelocity = 2000;
cfg.limits.maxMotorSpeed          = 8000;
motor.GetConfigurator().Apply(cfg);   // escribe 3 objetos, no 200
```

Además: `Save()` (`0x1010`, a memoria no volátil) y `RestoreDefaults()`
(`0x1011`).

18 tests.

#### `include/epos4/controls/ControlRequests.hpp` (160 líneas)

Seis modos de operación, encadenables:

```cpp
struct ProfilePosition { WithPosition(), WithVelocity() };
struct ProfileVelocity { WithVelocity() };
struct CyclicPosition  { WithPosition(), WithPositionOffset(), WithTorqueOffset() };
struct CyclicVelocity  { WithVelocity(), WithVelocityOffset(), WithTorqueOffset() };
struct CyclicTorque    { WithTorque(), WithTorqueOffset() };
struct Homing          { WithMethod(), ... };
struct Halt            { };
```

`Epos4::SetControl()` está sobrecargado para los siete. **El handshake de
setpoint de PPM (Tabla 3-15) se hace internamente**: poner target position,
subir bit 4 «New setpoint», esperar bit 12 «Setpoint acknowledge», bajar bit 4,
esperar que el drive baje bit 12. Sin ese baile el **segundo** movimiento no
ocurre — es *el* bug clásico de PPM, y aquí está resuelto dentro de la
librería.

#### `include/epos4/hardware/Encoder.hpp` + `src/hardware/Encoder.cpp`

Subsistema de realimentación aparte, accesible por `motor.GetEncoder()`.
Los cinco tipos de sensor: incremental digital 1 y 2, SinCos analógico, SSI
absoluto, Hall digital. Más la disposición de slots de `0x3000:01`.

**Detalle no obvio, ya resuelto:** la palabra de configuración de Hall tiene el
layout **espejo** del incremental (polaridad en bit 0, método en bit 4). Hay un
test que afirma que las dos codificaciones difieren, precisamente para que
nadie las "unifique" por error.

`Apply()` **se niega mientras el motor tenga potencia**, como exige el manual,
en vez de dejar que el drive aborte cada escritura y quede media configuración
aplicada.

13 tests.

#### `include/epos4/CanBus.hpp` + `src/CanBus.cpp`

```cpp
struct Options { std::string interface; std::string masterDcf; std::uint8_t masterNodeId; };
class CanBus {
  explicit CanBus(Options opts);
  void Start();          // arranca el hilo del bus y bootea la red
  void Stop();           // NO destruye el master (ver §16)
};
```

Posee el stack de Lely y corre el event loop **en su propio hilo**.

**Detalle crítico ya resuelto:** reescribe las rutas `UploadFile=` del
`master.dcf` a rutas absolutas y guarda `master.resolved.dcf`, porque Lely las
resuelve **en el boot**, no en la carga, y relativas fallan con `es='J'`.

#### `src/hardware/Epos4.cpp` (1434 líneas)

El grueso. `Epos4::Impl` deriva de **`canopen::LoopDriver`** de Lely.

**Por qué `LoopDriver` y no `FiberDriver`:** `FiberDriver` tiene que
construirse *en el hilo del bus*; construirlo fuera revienta con
`fiber_resume_with: Assertion 'curr' failed`. `LoopDriver` da llamadas
bloqueantes desde fuera del hilo, que es lo que la API síncrona necesita.

Las operaciones "lentas" (configurar, mover a poses) usan este patrón:

```cpp
std::promise<std::error_code> promise;
Defer([...]{ ... promise.set_value(...); });   // salto al hilo del bus
return future.get();                            // espera bloqueante
```

**Eso está bien para configurar y está MAL para un lazo de control.** De ahí la
Parte A (§8).

---

### 6.3 `epos4_ros2_control` — el plugin (terminado en esta sesión)

Ver §9 completo.

---

### 6.4 `epos4_bringup`

```
config/
├── epos4_network/          ← HARDWARE REAL
│   ├── bus.yml             79 líneas, muy comentado
│   └── epos4.eds           5754 líneas, exportado de EPOS Studio
└── sim_network/            ← SOLO para el mock de canopen_fake_slaves (NO epos4_sim, §10.3)
    ├── bus.yml             89 líneas
    └── sim_slave.eds       1442 líneas
launch/                     ⚠️ VACÍO
```

**Dos redes separadas, y es a propósito.** El master verifica la identidad del
esclavo (`0x1018`) contra `0x1F84`/`0x1F85` del DCF. El mock reporta vendor
`0x555` y un EPOS4 real reporta `0xFB`. Esa verificación es **comportamiento
correcto**, así que la simulación tiene su propio DCF en vez de debilitarla.

`generate_dcf()` (de `lely_core_libraries`) corre en el build y produce
`master.dcf`, `master.bin` y `node_2.bin` en `install/`.

**El mapeo PDO se declara en `bus.yml`**, no en código:

```yaml
node_2:
  node_id: 2
  dcf: "epos4.eds"
  heartbeat_producer: 500

  rpdo:                       # master → drive
    1:
      cob_id: "auto"
      transmission: 1         # 1 = aplicar en cada SYNC (esto hace CSP determinista)
      mapping:
        - {index: 0x6040}     # Controlword       16 bit
        - {index: 0x607A}     # Target position   32 bit
    2:
      transmission: 1
      mapping:
        - {index: 0x60FF}     # Target velocity   32 bit
        - {index: 0x6071}     # Target torque     16 bit

  tpdo:                       # drive → master
    1:
      transmission: 1
      mapping:
        - {index: 0x6041}     # Statusword        16 bit
        - {index: 0x6064}     # Position actual   32 bit
    2:
      transmission: 1
      mapping:
        - {index: 0x606C}     # Velocity actual   32 bit
        - {index: 0x6077}     # Torque actual     16 bit
```

**Por qué aquí y no en código:** los dos extremos tienen que estar de acuerdo
en qué bytes significan qué. Este es el único sitio que escribe ambos lados
desde una sola descripción. Remapear en caliente por SDO configuraría el drive
dejando el diccionario del master obsoleto.

**El `sim_network` mapea un subconjunto**, porque el EDS mock no tiene Target
torque (`0x6071`) ni Torque actual (`0x6077`); en su lugar el segundo TPDO
lleva Position demand (`0x6062`).

#### ⚠️ El acoplamiento SYNC ↔ interpolación

```yaml
sync_period: 10000    # MICROsegundos → 100 Hz
```

Tiene que cuadrar con **«Interpolation time period» (`0x60C2:01`)**, que el
driver fija vía `configs::CyclicConfigs::interpolationTimePeriodMs`.

**Ese objeto está en MILISEGUNDOS y este en MICROSEGUNDOS.** O sea:
`sync_period: 10000` ⟺ `interpolationTimePeriodMs = 10`.

Si no coinciden, el drive interpola sobre una ventana que no es aquella en la
que llegan los setpoints: muy pequeña y el movimiento va a saltos, muy grande y
va con retraso respecto a la trayectoria comandada. **Ninguno de los dos casos
falla de forma ruidosa.** Está comentado en los dos sitios.

---

### 6.5 `epos4_interfaces`

**Vacío.** Solo `CMakeLists.txt` + `package.xml`, sin `msg/` ni `srv/`.

El plugin **no lo necesita** — `ros2_control` ya publica `/joint_states` y
`/dynamic_joint_states`. Queda para cuando se quiera exponer diagnóstico
específico del EPOS4 (statusword decodificado, historial de errores,
servicios `FaultReset` / `Home` / `SetOperationMode`).

---

## 7. Conceptos ya cubiertos — NO RE-EXPLICAR

Imad pidió explícitamente ir despacio en CANopen:

> *"a ver vaquero, otra vez vas muy rápido sin asumir literal apenas sé qué es
> can, no sé qué son sus protocolos, el epoll, qué es un EMCY, un TPDO, lo del
> sync o qué es el bus off"*

Se retrocedió a cero y **ya se cubrió todo esto**. La siguiente sesión puede
darlo por sabido. Está aquí como referencia, no para volver a explicarlo.

### 7.1 CAN vs CANopen

- **CAN** es la capa física y de enlace: tramas con un identificador de 11 bits
  y hasta 8 bytes de datos. No hay direcciones de origen ni destino: **todo el
  mundo ve todas las tramas**.
- **Arbitraje**: si dos nodos transmiten a la vez, gana el del identificador
  **más bajo** (dominante). El perdedor se calla y reintenta. Por eso
  **conviene dar los node-ID bajos a lo que no puede quedarse sin ancho de
  banda**.
- **CANopen** (CiA 301) es un protocolo *encima* de CAN que le da significado a
  ese identificador.

### 7.2 COB-ID

```
COB-ID = código de función + node-ID
```

| Mensaje | Código de función | COB-ID del nodo 2 |
|---|---|---|
| NMT | `0x000` | `0x000` (broadcast) |
| SYNC | `0x080` | `0x080` (broadcast) |
| EMCY | `0x080` | `0x082` |
| TPDO1 (drive→master) | `0x180` | `0x182` |
| RPDO1 (master→drive) | `0x200` | **`0x202`** |
| TPDO2 | `0x280` | `0x282` |
| RPDO2 | `0x300` | `0x302` |
| SDO respuesta (tx) | `0x580` | `0x582` |
| SDO petición (rx) | `0x600` | `0x602` |
| Heartbeat | `0x700` | `0x702` |

**`0x202` es el COB-ID que hay que vigilar con `candump` para probar que el
camino de tiempo real funciona** (ver §13).

### 7.3 El diccionario de objetos

**Todo** en un drive CANopen es leer o escribir una celda de una tabla
indexada por `(índice de 16 bits, subíndice de 8 bits)`. No hay "comandos":
mover el motor es escribir en `0x607A`; arrancarlo es escribir en `0x6040`.

### 7.4 SDO vs PDO

| | SDO | PDO |
|---|---|---|
| Qué es | petición fiable con respuesta | difusión rápida sin confirmar |
| Tramas | 2 (petición + respuesta) | 1 |
| Direcciona | cualquier objeto, por índice | solo lo **premapeado** |
| Coste | round trip completo | ninguno una vez fluye |
| Para qué | configurar, arrancar, diagnosticar | el lazo de control |

**Un SDO lleva el índice dentro de la trama. Un PDO no**: los bytes van pelados
y el significado está en el mapeo acordado de antemano. Por eso el mapeo vive
en `bus.yml` y lo empuja el master en el boot.

### 7.5 Los cuatro mensajes de servicio

- **NMT** (Network Management): el master ordena a los nodos
  `Start` / `Stop` / `Pre-operational` / `Reset`. **Los PDOs solo circulan en
  estado `Operational`.**
- **SYNC**: un pulso que el master difunde. Cada nodo aplica el setpoint que
  recibió **en ese instante**, así todos los ejes actúan sincronizados en vez de
  desfasarse por lo que tardó cada trama. `transmission: 1` en `bus.yml` es
  exactamente esto.
- **EMCY**: el nodo grita un fallo **sin que nadie le pregunte**. Así una falta
  llega a la aplicación antes de que la siguiente lectura de estado la hubiera
  notado. En el driver: `SetEmergencyCallback()`.
- **Heartbeat**: cada nodo emite periódicamente "sigo vivo". Watchdog
  bidireccional: si el master deja de oír a un nodo, o el nodo al master,
  ambos pueden reaccionar.

### 7.6 Bus-off

Un nodo CAN lleva contadores de error de transmisión y recepción. Si el de
transmisión pasa de 255, el controlador **se desconecta solo del bus**
(*bus-off*) para no envenenar a los demás. Causas típicas: terminación mal
puesta (faltan los 120 Ω), baudrate distinto entre nodos, cableado. **Un nodo
en bus-off está mudo: no es que responda mal, es que no responde.**

### 7.7 La máquina de estados CiA 402

Un drive CiA 402 **no arranca escribiendo "muévete"**. Hay que recorrer:

```
  Not ready to switch on
          ↓  (automático)
  Switch on disabled
          ↓  Shutdown              (Controlword 0x0006)
  Ready to switch on
          ↓  Switch on             (Controlword 0x0007)
  Switched on
          ↓  Enable operation      (Controlword 0x000F)
  Operation enabled          ← aquí el motor tiene par
```

Y en paralelo: `Fault reaction active → Fault`, del que solo se sale con
**Fault reset**, que es **por flanco ascendente del bit 7**, no por nivel.

**Statusword, máscara `0x6F`.** Los comandos de la Tabla 2-7 llevan bits
*don't care* (`0xxx x110`); ignorarlos es el error clásico. Por eso `Decode()`
enmascara y `Controlword` hace read-modify-write preservando los bits de modo.

El manual insiste: *"a new state transition must not be initiated before the
previous one is completed"*. De ahí que `PlanStep()` avance **una transición
por ciclo** y que el drive reporte dónde está cada vez.

### 7.8 El modelo de concurrencia de Lely

- `io::Poll` + `ev::Loop`: un event loop sobre `epoll`.
- El loop **solo despierta por eventos encolados**. Si no se encola nada con
  `exec.post(...)`, se queda dormido y nunca vuelve — Imad dedujo esto solo:
  *"si yo quitara el exec post, no encolaría nada, entonces el loop tendría como
  muchas cosas pendientes y no volvería vdd"*. Correcto.
- `AsyncMaster` se conecta a un socket SocketCAN.
- `LoopDriver` vs `FiberDriver`: ver §6.2. Usamos **`LoopDriver`**.
- `Defer(...)` salta al hilo del bus; `rpdo_mapped` / `tpdo_mapped` son los
  arrays de valores mapeados; `OnRpdoWrite(idx, sub)` y `OnSync(cnt, time)` son
  los callbacks, **ambos en el hilo del bus**.

### 7.9 Modos de operación (`0x6060` / `0x6061`)

| Valor | Modo | Uso |
|---|---|---|
| 1 | **PPM** Profile Position | ir a poses; la rampa la genera el EPOS4 |
| 3 | **PVM** Profile Velocity | velocidad con rampa interna |
| 6 | **HMM** Homing | buscar el cero |
| 8 | **CSP** Cyclic Sync Position | **seguir trayectorias** ← lo del plugin |
| 9 | **CSV** Cyclic Sync Velocity | — |
| 10 | **CST** Cyclic Sync Torque | — |

### 7.10 Por qué el plugin usa CSP y no PPM

Esto se discutió y se decidió:

- **PPM genera la rampa *dentro* del EPOS4** y confirma por handshake. El
  encaje natural es un `JointGroupPositionController` (moverse a poses).
- **`JointTrajectoryController` asume que puedes comandar posición cada ciclo
  y que el drive sigue.** Eso es **CSP**, no PPM.
- Para un brazo que sigue trayectorias, **CSP es el modo correcto**.

Decisión cerrada: **el plugin comanda CSP, interfaz de comando `position`.**

---

## 8. Parte A — el camino de tiempo real (HECHO)

### 8.1 El problema que resuelve

Cada lectura y cada escritura del driver — **incluidas las que van por PDO** —
hacían esto:

```cpp
std::promise<std::error_code> promise;
Defer([...]{ ... promise.set_value(...); });   // salto de hilo
return future.get();                            // espera bloqueante
```

A 500 Hz, con 6 ejes y tres señales por eje, eso son **miles de cambios de
contexto por segundo en el hilo de control**, cada uno con una espera
bloqueante. Y peor: `SetControl` llamaba a `EnsureMode()`, que hace **una
lectura SDO por comando** — dos tramas CAN y un round trip, por eje, por ciclo.

Eso funciona para configurar y para mover a poses. **No funciona para
`read()`/`write()`.**

### 8.2 La solución

```
hilo de control                   hilo del bus (Lely)
───────────────                   ───────────────────
write() → atómicos  ──────────→   OnSync:      atómicos → tpdo_mapped
read()  ← atómicos  ←──────────   OnRpdoWrite: rpdo_mapped → atómicos
```

Sin mutex, sin syscall, sin salto de hilo. Solo cargas y almacenamientos
atómicos `relaxed`.

### 8.3 Los atómicos en `Epos4::Impl`

```cpp
std::atomic<std::int32_t>  cachedPosition;     // 0x6064
std::atomic<std::int32_t>  cachedVelocity;     // 0x606C
std::atomic<std::int16_t>  cachedTorque;       // 0x6077
std::atomic<std::uint16_t> cachedStatusword;   // 0x6041
std::atomic<std::int32_t>  stagedPosition;     // 0x607A, pendiente de SYNC
std::atomic<std::uint16_t> cachedControlword;  // 0x6040
std::atomic<bool>          cyclicActive;
```

### 8.4 `OnRpdoWrite` — entrada

```cpp
void OnRpdoWrite(std::uint16_t idx, std::uint8_t sub) noexcept override
{
  switch (idx) {
    case od::cia402::kPositionActualValue: {
      const std::int32_t value = rpdo_mapped[idx][0];
      cachedPosition.store(value, std::memory_order_relaxed);
      break;
    }
    // ... velocidad, par, statusword
  }
}
```

> **Detalle de la API de Lely que costó descubrir:** `rpdo_mapped[idx][sub]`
> **convierte implícitamente al asignar**. No existe `.Read<T>()`. El tipo lo
> decide la variable de destino.

### 8.5 `OnSync` — salida

```cpp
void OnSync(std::uint8_t, const time_point &) noexcept override
{
  if (!cyclicActive.load(std::memory_order_relaxed)) { return; }
  try {
    tpdo_mapped[od::cia402::kControlword][0]    = cachedControlword.load(std::memory_order_relaxed);
    tpdo_mapped[od::cia402::kTargetPosition][0] = stagedPosition.load(std::memory_order_relaxed);
  } catch (...) {
    // el objeto no está mapeado en este DCF; no es fatal
  }
}
```

`OnSync` se ejecuta en el hilo del bus **justo cuando toca transmitir**. Es el
sitio correcto para publicar.

### 8.6 La API pública añadida a `Epos4`

```cpp
// --- camino cíclico de tiempo real ---

// Fija el modo UNA vez (no por ciclo) y activa la publicación en SYNC.
std::error_code EnterCyclicPositionMode();
void            ExitCyclicMode();
bool            IsCyclicModeActive() const;

// Lock-free. Seguros dentro de un lazo de tiempo real.
std::int32_t  GetCachedPosition()   const;  // 0x6064 [quadcounts]
std::int32_t  GetCachedVelocity()   const;  // 0x606C [rpm]
std::int16_t  GetCachedTorque()     const;  // 0x6077 [por mil del par nominal]
std::uint16_t GetCachedStatusword() const;  // 0x6041

// Escenifica el siguiente objetivo. Sale en el SYNC siguiente.
// Lock-free y seguro desde un hilo distinto al del bus.
void StageTargetPosition(std::int32_t quadCounts);

// True mientras llegan PDOs Y el drive reporta «Operation enabled».
bool IsCyclicHealthy(
  std::chrono::steady_clock::duration maxAge = std::chrono::milliseconds{50}) const;
```

### 8.7 Dos detalles que no son obvios

**1. `EnterCyclicPositionMode()` siembra `stagedPosition` con la posición
actual.** Si no, el primer SYNC comanda un movimiento al origen y el brazo
salta.

**2. `IsCyclicHealthy()` hay que mirarlo cada ciclo.** Si el bus se calla, los
valores cacheados **dejan de cambiar pero siguen leyéndose tan felices**, y el
controlador seguiría creyendo que un eje muerto está siguiendo la trayectoria.
`maxAge` por defecto son 50 ms, que con SYNC de 10 ms es generoso.

### 8.8 Estado

✅ **Implementado, compilando y verificado de extremo a extremo.**
✅ **Testeado sin bus** vía `core::CyclicState` (§11, paso 3).

---

## 9. Parte B — el plugin de ros2_control (HECHO)

### 9.1 Archivos

```
epos4_ros2_control/
├── include/epos4_ros2_control/Epos4System.hpp    101 líneas
├── src/Epos4System.cpp                           362 líneas
├── epos4_ros2_control_plugin.xml                  11 líneas
├── test/test_epos4_system.cpp                    168 líneas, 12 tests
├── CMakeLists.txt                                 76 líneas
└── package.xml                                    36 líneas
```

### 9.2 Contrato

```
interfaz de comando:   position                  (rad en la articulación)
interfaces de estado:  position, velocity, effort (rad, rad/s, Nm)
modo del drive:        CSP (Cyclic Synchronous Position)
```

Compatible con `joint_trajectory_controller` y
`position_controllers/JointGroupPositionController`.

**Todo en radianes**, como espera `ros2_control`. La conversión a cuentas la
hace `epos4::MechanismScale`, configurada por articulación desde el URDF.

### 9.3 La estructura `Axis`

```cpp
struct Axis
{
  std::string name;
  std::uint8_t nodeId{0};
  epos4::MechanismScale scale;
  std::unique_ptr<epos4::Epos4> device;   // puntero: Epos4 no es copiable ni movible

  // Se entregan a ros2_control POR DIRECCIÓN: no pueden moverse una vez
  // exportadas las interfaces. El vector se dimensiona una sola vez en on_init.
  double statePosition{0.0};    // rad
  double stateVelocity{0.0};    // rad/s
  double stateEffort{0.0};      // Nm
  double commandPosition{0.0};  // rad

  std::uint32_t ratedTorqueMicroNm{0};
};
```

> ⚠️ **`axes_` se dimensiona en `on_init` y NO puede reasignarse después.**
> `export_state_interfaces()` entrega punteros a estos `double`. Si el vector
> crece y reubica, ros2_control escribe en memoria liberada.

### 9.4 El ciclo de vida

| método | qué hace |
|---|---|
| `on_init` | Lee y **valida** los parámetros del URDF. **No toca el bus.** |
| `on_configure` | Crea el `CanBus`, construye un `Epos4` por joint **antes de `Start()`**, arranca, `WaitUntilReady()`, lee el par nominal. |
| `on_activate` | `Enable()`, `EnterCyclicPositionMode()`, **siembra el comando con la posición actual**. |
| `on_deactivate` | `ExitCyclicMode()` + `Disable()`. **Completa aunque haya un eje en fault.** |
| `on_cleanup` | Suelta el bus y los dispositivos. |
| `read` | Cargas atómicas + conversión. **Cero bus.** |
| `write` | Conversión + `StageTargetPosition`. **Cero bus.** |

### 9.5 Lo que valida `on_init` (y por qué)

Los 12 tests cubren exactamente esto:

```
AcceptsAWellFormedConfiguration
ExportsOneCommandAndThreeStateInterfacesPerJoint
RequiresTheMasterDcf
RejectsACommandInterfaceItCannotDrive
RejectsMoreThanOneCommandInterface
RejectsAStateInterfaceItDoesNotProvide
RejectsDuplicateNodeIds
RequiresTheEncoderResolution
RejectsAZeroGearRatio
RejectsUnparseableParameters
RequiresANodeIdPerJoint
RadiansConvertToCountsThroughTheGearbox
```

Cada uno de estos rechazos evita un fallo que en runtime sería confuso:
un node-ID duplicado hace que dos joints compartan drive silenciosamente; un
gear ratio de 0 hace una división por cero dentro de `MechanismScale`.

### 9.6 `on_configure` — el orden importa

```cpp
bus_ = std::make_unique<epos4::CanBus>(
  epos4::CanBus::Options{canInterface_, masterDcf_, masterNodeId_});

// Devices BEFORE Start(). A CANopen master boots each slave once, at
// reset, and only routes a node's PDOs to a driver registered at that
// moment. Constructing them afterwards gives nodes that answer SDO
// perfectly and never deliver a single PDO.
for (auto & axis : axes_) {
  axis.device = std::make_unique<epos4::Epos4>(*bus_, axis.nodeId);
  axis.device->SetMechanism(
    axis.scale.GetQuadCountsPerRevolution(), axis.scale.GetGearRatio());
}

bus_->Start();
```

**Este es uno de los bugs más caros de la sesión** (ver §16): los dispositivos
tienen que existir **antes** de `Start()`, o responden SDO perfectamente y no
entregan ni un PDO.

Después, por cada eje: `WaitUntilReady()` y lectura única del par nominal
(`GetMotorRatedTorque()`). Si el par nominal es 0 (datos del motor sin
configurar), **la interfaz de effort reporta 0** en vez de un número escalado
por un desconocido, y se avisa con un `RCLCPP_WARN`.

### 9.7 `on_activate` — la siembra

```cpp
if (!axis.device->Enable()) { /* ERROR */ }
if (auto ec = axis.device->EnterCyclicPositionMode()) { /* ERROR */ }

// Seed the command with where the axis already is. Without this the first
// write() sends the default command of zero and the arm swings to its
// origin the instant the controller activates.
const double position =
  axis.scale.ToAngle(axis.device->GetCachedPosition()).value() * 2.0 * M_PI;
axis.statePosition   = position;
axis.commandPosition = position;
```

### 9.8 `read()`

```cpp
for (auto & axis : axes_) {
  // Relaxed atomic loads. No bus traffic, no locking, no thread hop.
  const std::int32_t counts      = axis.device->GetCachedPosition();
  const std::int32_t rpm         = axis.device->GetCachedVelocity();
  const std::int16_t perThousand = axis.device->GetCachedTorque();

  axis.statePosition = axis.scale.ToAngle(counts).value() * 2.0 * M_PI;
  axis.stateVelocity = units::angular_velocity::radians_per_second_t(
                         axis.scale.ToAngularVelocity(rpm)).value();
  axis.stateEffort   = epos4::ToTorque(perThousand, axis.ratedTorqueMicroNm).value();

  if (!axis.device->IsCyclicHealthy()) { healthy = false; /* log una vez */ }
}
if (!healthy) { return hardware_interface::return_type::ERROR; }
```

El log de "no saludable" **se emite una sola vez** (`unhealthyReported_`): un
bus caído produciría si no un mensaje por eje y por ciclo, enterrando todo lo
demás.

### 9.9 `write()`

```cpp
for (auto & axis : axes_) {
  if (!std::isfinite(axis.commandPosition)) {
    // A NaN would convert to an arbitrary count and command the axis
    // somewhere unrelated. Hold the last good setpoint instead.
    continue;
  }
  const units::angle::turn_t target{axis.commandPosition / (2.0 * M_PI)};
  axis.device->StageTargetPosition(axis.scale.ToQuadCounts(target));
}
```

### 9.10 El export de pluginlib

Tres cosas tienen que coincidir o falla **en runtime** con "class not found",
no en compilación:

1. `src/Epos4System.cpp`, al final:
   ```cpp
   PLUGINLIB_EXPORT_CLASS(
     epos4_ros2_control::Epos4System, hardware_interface::SystemInterface)
   ```
2. `epos4_ros2_control_plugin.xml`:
   ```xml
   <library path="epos4_ros2_control">
     <class name="epos4_ros2_control/Epos4System"
            type="epos4_ros2_control::Epos4System"
            base_class_type="hardware_interface::SystemInterface">
   ```
3. `package.xml`:
   ```xml
   <export>
     <hardware_interface plugin="${prefix}/epos4_ros2_control_plugin.xml"/>
   </export>
   ```

Y en `CMakeLists.txt`, la librería se instala en **`lib/`**, no en
`lib/${PROJECT_NAME}/`, porque pluginlib hace `dlopen` desde el *library path*,
no desde el path del paquete.

### 9.11 Estado

✅ Escrito, compilando, 12 tests en verde.
✅ **Cargado por `controller_manager` y verificado contra `epos4_sim`** (§11 pasos 2 y 4).

---

## 10. Parte C — el simulador y el bloqueo del remapeo PDO

### 10.1 Por qué existe `epos4_sim`

`ros-humble-canopen-fake-slaves` trae un slave CANopen simulado, pero **su mock
responde `0x0040` a cualquier escritura del Controlword y nunca avanza de
estado**. Con eso la secuencia de habilitación es intesteable.

`src/epos4_driver/tools/epos4_sim.cpp` (493 líneas) es un slave CANopen que:

- Implementa las transiciones CiA 402 **de verdad**.
- **Carga el EDS real de maxon**, así que la verificación de identidad pasa y
  se puede usar la configuración de producción contra él.
- Modela movimiento con un temporizador.

```cpp
class Epos4Sim : public canopen::BasicSlave
{
  void HandlePpm(std::uint16_t cw);   // handshake bit 4 ↔ bit 12
  void HandleCsp();                   // ← añadido en esta sesión
  void OnWrite(std::uint16_t idx, std::uint8_t subidx) noexcept override;
  void PublishStatusword();

  bool         cspFollowing_{false};
  std::int32_t interpolationMs_{10};
};
```

### 10.2 CSP en el simulador (añadido en esta sesión)

- `HandleCsp()` se dispara desde `OnWrite` tanto en `0x6040` (Controlword) como
  en `0x607A` (Target position).
- Lee el periodo de interpolación de `0x60C2:01` y **acota el paso por ese
  periodo**, en vez de usar la rampa de PPM:
  ```cpp
  interpolationMs_ = (periodMs == 0) ? 1 : periodMs;
  ...
  cspFollowing_ ? (interpolationMs_ > kTickMs ? interpolationMs_ : kTickMs) : ...
  ```
- Publica el **bit 12 «Drive follows command value»**
  (`sig::status_bits::kFollowsCommandValue`) mientras está siguiendo.
- Limpia `cspFollowing_` al salir del modo.

### 10.3 ✅ EL "BLOQUEO" — RESUELTO (2026-09-24, sesión siguiente): ERA UN DIAGNÓSTICO EQUIVOCADO

> **Lo de abajo es FALSO y se deja tachado como registro.** `BasicSlave` SÍ
> aplica el mapeo que descarga el master. Verificado con una sonda en memoria
> (`io::VirtualCanController`, sin `vcan0` ni sudo): master con el DCF de
> `epos4_network` + esclavo con `epos4.eds` → `0x1600/0x1601/0x1A00/0x1A01`
> quedan exactamente como `bus.yml`, 299 TPDOs de cada objeto y 299 escrituras
> RPDO de `0x607A` en 3 s (100 Hz).
>
> **Causa real:** `epos4_sim` fuerza la identidad maxon (`SetIdentity()`,
> vendor `0xFB`) y se estaba lanzando contra el DCF de **`sim_network`**, que
> espera `0x555` (el mock de `canopen_fake_slaves`). El master aborta con
> `es='D'` **antes** de descargar `node_2.bin`, el simulador se queda con el
> mapeo por defecto del EDS, y el master sigue decodificando esas tramas con
> *su* mapeo → basura con pinta de dato. Reproducido con la misma sonda.
>
> **Arreglo:** `epos4_sim` va SIEMPRE con `epos4.eds` + `epos4_network`.
> `sim_network` solo sirve para el mock de `canopen_fake_slaves`. No hay que
> tocar ni el EDS ni el simulador.

~~Texto original:~~

> **`BasicSlave` de Lely acepta las escrituras SDO del mapeo PDO pero NO
> remapea sus PDOs en caliente.** Se queda con el mapeo del EDS.

Consecuencia: el master empuja la configuración concisa (`node_2.bin`) en el
boot, el simulador la acepta sin quejarse, y **sigue transmitiendo con el mapeo
del EDS**. Los dos extremos creen cosas distintas sobre qué bytes significan
qué.

**Esto es lo que impide verificar CSP de extremo a extremo.** También es lo que
ya impedía verificar PPM (está anotado en el CHANGELOG como limitación
conocida).

**Qué hay que hacer:** que el simulador **aplique el mapeo al arrancar**,
leyendo los objetos `0x1600`/`0x1601` (RPDO) y `0x1A00`/`0x1A01` (TPDO) de su
propio diccionario después de que el master los haya escrito, y reconfigurando
sus PDOs en consecuencia.

Pistas de implementación:
- Sobrescribir `OnWrite` para `0x1600`–`0x1601` y `0x1A00`–`0x1A01` y
  reconstruir el mapeo cuando cambie el subíndice 0 (el contador de entradas).
- O bien aplicar el mapeo una vez al recibir la transición NMT a `Operational`.
- La alternativa perezosa: **alinear el EDS del simulador
  (`sim_slave.eds`) con el mapeo de `sim_network/bus.yml`**, de forma que el
  mapeo por defecto del EDS ya sea el correcto y no haga falta remapear. Es
  menos elegante pero desbloquea la verificación hoy.

> 💡 **Recomendación:** empezar por la alternativa perezosa (editar
> `sim_slave.eds` para que sus `0x1600`/`0x1A00` por defecto coincidan con
> `bus.yml`). Desbloquea §13 en una tarde. El remapeo dinámico correcto puede
> venir después.

---

## 11. Siguientes pasos, en orden

### Paso 1 — ~~Desbloquear el mapeo PDO del simulador~~ ✅ HECHO (no había bloqueo, ver §10.3)

**Verificado sobre `vcan0` con la librería real** (`api_demo` + `epos4_sim`
con `epos4_network`): los cuatro PDOs (`182`, `282`, `202`, `302`) salen con
6 bytes, `pdo: active`, y un movimiento PPM de 50000 cuentas avanza por TPDO.
Por el camino aparecieron y se arreglaron dos bugs (ver §16, las dos filas
de PPM).

**Por qué primero:** sin esto no se puede verificar nada de lo demás.

**Qué hacer:** ver §10.3. Recomendado: alinear `sim_slave.eds` con
`sim_network/bus.yml`.

**Criterio de aceptación:**
```bash
candump vcan0 | grep -E ' 18[23] | 28[23] '
# deben verse TPDOs del nodo 2 con el contenido esperado
```
y en la aplicación, `motor.IsPdoActive()` devuelve `true` y
`GetCachedPosition()` cambia al moverse.

---

### Paso 2 — Escribir los archivos de bringup ✅ HECHO

Son cuatro, no tres: `ros2_control_node` necesita un URDF completo, así que
además de la macro hay un URDF mínimo de un eje.

```
config/epos4.ros2_control.xacro   macros epos4_system + epos4_joint (genéricas)
config/epos4_arm.urdf.xacro       base_link + joint 'shoulder' (node 2, placeholders)
config/controllers.yaml           100 Hz, JSB + arm_controller (+ JointGroupPosition opcional)
launch/arm.launch.py              arg can_interface (vcan0/can0), master_dcf, description
```

El launch ya no tiene argumento `sim`: simulador y hardware usan el mismo DCF
(`epos4_network`), solo cambia `can_interface`. **Verificado contra
`epos4_sim`**; al hacerlo aparecieron el segfault de `SetMechanism()` y el
temporizador del simulador que no avanzaba en CSP (ver §16).

~~Plan original:~~

```
src/epos4_bringup/config/epos4_arm.ros2_control.xacro
src/epos4_bringup/config/controllers.yaml
src/epos4_bringup/launch/arm.launch.py
```

Plantillas completas en **§12**. `launch/` ya existe como directorio vacío y
`CMakeLists.txt` ya lo instala (`install(DIRECTORY config launch ...)`), así
que no hay que tocar CMake.

**Criterio de aceptación:** `colcon build` sigue verde y los archivos aparecen
en `install/epos4_bringup/share/epos4_bringup/`.

---

### Paso 3 — Tests del camino cíclico en el driver ✅ HECHO (2026-09-24)

Ni test-friend ni constructor de test: el estado cíclico se sacó a
**`core::CyclicState`** (`include/epos4/core/CyclicState.hpp`, en
`epos4_core`, sin Lely), con el reloj inyectado. `Impl` lo alimenta desde
`OnRpdoWrite`/`OnSync`; `test_cyclic_state` (15 tests) lo alimenta a mano.
Comprobado por mutación: quitar la comprobación de edad hace fallar 3 tests.
Re-verificado de extremo a extremo en `vcan0` tras el refactor (−0,4 rad
exactos).

Al hacerlo salieron dos bugs (ver §16): orden de memoria del flag «active»
(podía publicarse un 0 en el primer SYNC en ARM) y `QuickStop()`/`Halt()`
pisados por `OnSync` durante el modo cíclico.

~~Plan original:~~

Están en el plan aprobado y **no se escribieron**. En `epos4_driver`, sin bus:

- Los cachés devuelven lo último escrito.
- **`IsCyclicHealthy()` se vuelve falso cuando los PDOs dejan de llegar.**
  (Este es el importante: es la red de seguridad del plugin.)
- `StageTargetPosition` no bloquea y es seguro desde otro hilo.

**Dificultad:** los atómicos viven en `Epos4::Impl`, que es privado. Opciones:
(a) un test-friend, (b) exponer un constructor de test que no adjunte a un bus,
(c) testear a través de la API pública arrancando el simulador. (a) o (b) son
preferibles: el plan dice **"sin bus"**.

**Criterio de aceptación:** 3+ tests nuevos en verde, sin `vcan0` levantado.

---

### Paso 4 — Verificación end-to-end contra el simulador ✅ HECHO (2026-09-24)

Trayectoria a 0,5 rad en 3 s: el RPDO `0x202` lleva una Target position nueva
en cada SYNC, sin SDO durante el movimiento; la posición sigue ~100 cuentas
(≈3 mrad) por detrás y llega a 15915 cuentas = 0,500 rad en ~3,2 s. Cero
errores de tolerancia. 

**§13.7 (red de seguridad) también verificado:** matar el simulador con el
controlador activo → detección en ~64 ms (umbral de 50 ms + un ciclo), un
solo log sin tocar el bus, `on_error` desactiva el camino cíclico y el RPDO
pasa a llevar `0x000D` («Disable voltage») en cada SYNC. Nota: en Humble los
controladores siguen figurando `active` tras el error del hardware; es
comportamiento de ros2_control Humble, no del plugin.

~~Plan original:~~

Receta completa en **§13**.

**Criterio de aceptación**, literal del plan aprobado:

> Con `candump vcan0` en una cuarta terminal debería verse el RPDO **`0x202`
> cambiando en cada SYNC**, no ráfagas de SDO — que es la prueba de que el
> camino de tiempo real está funcionando y no cayendo al de reserva.

---

### Paso 5 — Limpieza ✅ HECHO (2026-09-24)

Borrados `src/epos4_driver/{install,build,log}`. Build **desde cero**
(`rm -rf build install log`) sin un solo aviso, tras quitar dos `-Wcomment`
(comentarios `//` terminados en `\`) y los inicializadores designados C++20
de `api_demo`. uncrustify limpio, sin headers de ROS en `epos4_driver`, sin
direcciones hex desnudas. **213 tests, 0 fallos.** Los commits, pendientes
de Imad.

~~Plan original:~~

```bash
rm -rf src/epos4_driver/install src/epos4_driver/build src/epos4_driver/log
source /opt/ros/humble/setup.bash
ament_uncrustify --reformat src/epos4_driver src/epos4_ros2_control
colcon build && colcon test && colcon test-result --all
```

Mensajes de commit en **§17**. **Los ejecuta Imad, no la sesión.**

---

### Paso 6 — Actualizar CHANGELOG y README ✅ HECHO (2026-09-24)

CHANGELOG: sección `[Unreleased]` con todo lo de esta sesión. README:
estado, tabla de paquetes, secciones nuevas «Under ros2_control» y «When
something dies», `bus.yml` con heartbeat, ejemplos en C++17, limitaciones
al día.

~~Plan original:~~

El `CHANGELOG.md` de `epos4_driver` sigue diciendo:

> *"The `epos4_ros2_control` and `epos4_interfaces` packages are empty."*

**Ya no es cierto para `epos4_ros2_control`.** Hay que añadir una entrada
`[0.2.0]` con el camino cíclico y el plugin, y corregir las limitaciones.

El `README.md` (sin commitear todavía) tampoco menciona el plugin.

---

### Paso 7 — Hardware real

Imad **sí tiene acceso al EPOS4**, aunque no de forma continua.

Antes de conectar:

1. **Confirmar el modelo** (Module 50/15 vs Compact 50/5, §3). Si es Compact,
   hay que exportar el EDS correcto de EPOS Studio y regenerar el DCF.
2. Verificar terminación del bus (**120 Ω en los dos extremos**).
3. Verificar que el baudrate del drive (`0x2001`) coincide con
   `baudrate: 1000` de `bus.yml`.
4. Verificar node-IDs (`0x2000`) sin duplicados.
5. **Ejecutar primero `examples/api_demo.cpp`**, no el plugin. Si el driver no
   habla con el drive, el plugin tampoco.
6. Empezar con **un solo eje** y sin carga mecánica.

---

## 12. Plantillas de los archivos que faltan

### 12.1 `src/epos4_bringup/config/epos4_arm.ros2_control.xacro`

> ⚠️ Los valores de `quadcounts_per_revolution` y `gear_ratio` son de ejemplo.
> **Hay que poner los reales del brazo de Quantum.**
> `quadcounts_per_revolution` = CPR del encoder × 4. Un encoder de 500 CPR da
> **2000**. `gear_ratio` es la relación salida/motor: una reductora 1:100 es
> **0.01**.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ===================================================================
       ros2_control description for an arm of maxon EPOS4 drives on one
       CAN bus.

       All joint values are in RADIANS, as ros2_control expects. The
       conversion to encoder quadcounts happens inside the plugin, from
       quadcounts_per_revolution and gear_ratio.

       KEEP master_dcf IN STEP with which network was generated:
         epos4_network  real hardware
         sim_network    the canopen_fake_slaves mock ONLY (epos4_sim uses epos4_network)
       =================================================================== -->
  <xacro:macro name="epos4_arm_ros2_control"
               params="name:=epos4_arm
                       can_interface:=can0
                       master_dcf
                       master_node_id:=1">

    <ros2_control name="${name}" type="system">

      <hardware>
        <plugin>epos4_ros2_control/Epos4System</plugin>
        <param name="can_interface">${can_interface}</param>
        <param name="master_dcf">${master_dcf}</param>
        <param name="master_node_id">${master_node_id}</param>
      </hardware>

      <!-- One block per axis. The node_id must match bus.yml. -->
      <joint name="shoulder">
        <param name="node_id">2</param>
        <param name="quadcounts_per_revolution">2000</param>
        <param name="gear_ratio">0.01</param>
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>

      <!--
      <joint name="elbow">
        <param name="node_id">3</param>
        <param name="quadcounts_per_revolution">2000</param>
        <param name="gear_ratio">0.01</param>
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
      -->

    </ros2_control>
  </xacro:macro>
</robot>
```

> **Nota:** `ros2_control` necesita que el URDF también declare los `<link>` y
> `<joint>` correspondientes. Si el brazo ya tiene una descripción URDF en otro
> paquete, este macro se incluye desde ahí. Si no, hace falta un URDF mínimo
> con un link por articulación para que `robot_state_publisher` no proteste.

---

### 12.2 `src/epos4_bringup/config/controllers.yaml`

```yaml
# ===========================================================================
# controller_manager configuration.
#
# update_rate is the frequency of the read()/write() loop. It should divide
# evenly into the SYNC period of bus.yml (sync_period: 10000 us = 100 Hz),
# so that every control cycle lines up with a SYNC instead of drifting
# against it.
#
#   sync_period 10000 us  ->  100 Hz  ->  update_rate: 100
# ===========================================================================
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    # Follows trajectories. This is what CSP exists for.
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    # Alternative: go to poses, no trajectory. Useful for bring-up and
    # for manual jogging. Do NOT activate both at once - they claim the
    # same command interfaces and the second one will be rejected.
    # arm_position_controller:
    #   type: position_controllers/JointGroupPositionController


joint_state_broadcaster:
  ros__parameters:
    use_local_topics: false


arm_controller:
  ros__parameters:
    joints:
      - shoulder
      # - elbow

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    # Interpolate between trajectory points rather than stepping.
    allow_partial_joints_goal: false
    open_loop_control: false

    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.5
      shoulder:
        trajectory: 0.15   # rad, max deviation during the move
        goal: 0.05         # rad, max error at the end


# arm_position_controller:
#   ros__parameters:
#     joints:
#       - shoulder
```

---

### 12.3 `src/epos4_bringup/launch/arm.launch.py`

```python
# ===========================================================================
# Brings up the EPOS4 arm under ros2_control.
#
#   ros2 launch epos4_bringup arm.launch.py                 # simulator
#   ros2 launch epos4_bringup arm.launch.py sim:=false \
#        can_interface:=can0                                # real hardware
#
# The DCF is picked from the installed share directory, which is where
# generate_dcf() puts it at build time. Passing a relative path would fail:
# Lely resolves UploadFile paths at BOOT, not at load.
# ===========================================================================

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory('epos4_bringup')

    sim = LaunchConfiguration('sim')
    can_interface = LaunchConfiguration('can_interface')

    # ---- the two networks generate_dcf() produced at build time ----------
    # epos4_sim answers with the maxon identity, so it runs on the SAME
    # network as the hardware. sim_network is for the canopen_fake_slaves
    # mock only: against it the boot aborts with es='D' before the PDO
    # mapping is downloaded.
    sim_dcf = os.path.join(share, 'config', 'epos4_network', 'master.dcf')
    hw_dcf = os.path.join(share, 'config', 'epos4_network', 'master.dcf')

    controllers = os.path.join(share, 'config', 'controllers.yaml')

    # ---- robot_description ----------------------------------------------
    # The xacro takes the DCF path so the same description serves both
    # the simulator and the real bus.
    robot_description_sim = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('epos4_bringup'),
            'config', 'epos4_arm.ros2_control.xacro']),
        ' can_interface:=', can_interface,
        ' master_dcf:=', sim_dcf,
    ])

    robot_description_hw = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('epos4_bringup'),
            'config', 'epos4_arm.ros2_control.xacro']),
        ' can_interface:=', can_interface,
        ' master_dcf:=', hw_dcf,
    ])

    control_node_sim = Node(
        package='controller_manager',
        executable='ros2_control_node',
        condition=IfCondition(sim),
        parameters=[{'robot_description': robot_description_sim}, controllers],
        output='screen',
    )

    control_node_hw = Node(
        package='controller_manager',
        executable='ros2_control_node',
        condition=UnlessCondition(sim),
        parameters=[{'robot_description': robot_description_hw}, controllers],
        output='screen',
    )

    # ---- spawners --------------------------------------------------------
    # The broadcaster goes first: if the arm controller comes up before the
    # hardware is active, its claim on the command interfaces is rejected.
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='true',
            description='Expect epos4_sim on vcan0 (same epos4_network DCF as hardware)'),
        DeclareLaunchArgument(
            'can_interface', default_value='vcan0',
            description='SocketCAN interface name'),

        control_node_sim,
        control_node_hw,
        jsb_spawner,

        # Chain the arm controller behind the broadcaster.
        RegisterEventHandler(
            OnProcessExit(target_action=jsb_spawner,
                          on_exit=[arm_spawner])),
    ])
```

> ⚠️ **Dos cosas que verificar al escribirlo de verdad:**
> 1. `master.dcf` está **gitignored** y lo genera el build — existe en
>    `install/`, no en `src/`. La ruta de arriba apunta a `share/`, que es la
>    correcta.
> 2. Si el xacro no existe todavía, `Command(['xacro ', ...])` falla con un
>    error poco claro. Escribir primero el §12.1.

---

## 13. Receta de verificación end-to-end

### 13.1 Preparación (una vez por reinicio)

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
ip link show vcan0
```

### 13.2 Build

```bash
cd /data/epos_controller_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 13.3 Cuatro terminales

**Terminal 1 — el simulador**

```bash
cd /data/epos_controller_ws
source install/setup.bash
# epos4_network, NO sim_network (ver §10.3)
CFG=install/epos4_bringup/share/epos4_bringup/config/epos4_network
./build/epos4_driver/epos4_sim $CFG/epos4.eds 2 vcan0
```

**Terminal 2 — el tráfico crudo** (dejarlo corriendo desde el principio)

```bash
candump vcan0
```

**Terminal 3 — el controlador**

```bash
cd /data/epos_controller_ws
source install/setup.bash
ros2 launch epos4_bringup arm.launch.py
```

**Terminal 4 — comprobaciones**

```bash
source /data/epos_controller_ws/install/setup.bash

ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

### 13.4 Qué debe salir

**`ros2 control list_hardware_interfaces`:**

```
command interfaces
        shoulder/position [available] [claimed]
state interfaces
        shoulder/effort
        shoulder/position
        shoulder/velocity
```

`[claimed]` significa que `arm_controller` se quedó con la interfaz de comando.
Si dice `[available]` sin `[claimed]`, el controlador no arrancó.

**`ros2 control list_controllers`:**

```
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster  active
arm_controller          joint_trajectory_controller/JointTrajectoryController  active
```

### 13.5 Mover el eje

```bash
# -w 1: espera a que haya un suscriptor. Sin él, --once a veces publica y
# sale antes del descubrimiento DDS, y parece que el eje no responde.
ros2 topic pub -w 1 --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory "{
    joint_names: ['shoulder'],
    points: [
      { positions: [1.0], time_from_start: { sec: 3 } }
    ]
  }"
```

Y observar:

```bash
ros2 topic echo /joint_states
```

`position` debe subir progresivamente de su valor inicial hacia `1.0` rad.

### 13.6 ⭐ LA PRUEBA QUE IMPORTA

En la terminal 2 (`candump`), filtrar el RPDO del nodo 2:

```bash
candump vcan0 | grep ' 202 '
```

**Debe verse:**

```
  vcan0  202   [6]  0F 00 A1 05 00 00
  vcan0  202   [6]  0F 00 B4 05 00 00
  vcan0  202   [6]  0F 00 C7 05 00 00
  ...
```

- **Cada 10 ms** (el `sync_period`), de forma regular.
- Los dos primeros bytes son el **Controlword** (`0x000F` = Operation
  enabled).
- Los cuatro siguientes son la **Target position**, y **tienen que ir
  cambiando**.

**Si en cambio se ven ráfagas en `602`/`582` (SDO), el camino de tiempo real
NO está funcionando** y algo cayó al camino de reserva. Eso es exactamente lo
que este trabajo existe para evitar.

### 13.7 Comprobar la red de seguridad

Matar el simulador (Ctrl-C en la terminal 1) con el controlador activo.

**Esperado:** en menos de ~50 ms, `read()` devuelve `ERROR`, aparece **una
sola vez** el log:

```
[ERROR] joint 'shoulder' stopped delivering PDOs or left «Operation enabled». ...
```

y `controller_manager` desactiva el hardware. **No** debe verse el mensaje
repetido cada ciclo, ni el controlador debe seguir tan tranquilo creyendo que
el eje sigue la trayectoria.

---

## 14. Estado de build y tests (números reales)

Ejecutado el **2026-09-24**, al cerrar esta sesión:

```bash
cd /data/epos_controller_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install     # 5 packages finished
colcon test                        # 5 packages finished
colcon test-result --all
```

**Resultado: `Summary: 184 tests, 0 errors, 0 failures, 0 skipped`** ✅

> **Actualizado (sesión siguiente, build desde cero):** `213 tests, 0 errors, 0 failures`. Nuevos: `test_cyclic_state` (15), `test_device_lifecycle` (4), más linters de los archivos nuevos.

Desglose:

| Paquete | Suite | Tests | Estado |
|---|---|---:|---|
| `robot_units` | `test_units` | 8 | ✅ |
| | `lint_cmake`, `xmllint` | 2 | ✅ |
| `epos4_driver` | `test_cia402_state_machine` | 18 | ✅ |
| | `test_configs` | 18 | ✅ |
| | `test_digital_inputs` | 14 | ✅ |
| | `test_encoder_configs` | 13 | ✅ |
| | `test_errors` | 14 | ✅ |
| | `test_unit_conversion` | 18 | ✅ |
| | `uncrustify` | 35 | ✅ |
| | `lint_cmake`, `xmllint` | 2 | ✅ |
| `epos4_ros2_control` | `test_epos4_system` | **12** | ✅ |
| | `uncrustify` | 3 | ✅ |
| | `lint_cmake`, `xmllint` | 3 | ✅ |
| `epos4_bringup` | `lint_cmake`, `xmllint` | 2 | ✅ |
| `epos4_interfaces` | `lint_cmake`, `xmllint` | 2 | ✅ |

**Tests gtest reales: 95 (driver) + 12 (plugin) + 8 (units) = 115.**
El resto son linters.

> **Ninguno de estos tests necesita un bus ni hardware.** Eso es deliberado y
> hay que mantenerlo.

### Nota sobre los linters

En `epos4_driver`, `epos4_ros2_control` y `robot_units` se sustituyó
`ament_lint_auto` por linters **explícitamente acotados**:

```cmake
ament_uncrustify(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)
ament_lint_cmake(${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt)
ament_xmllint(${CMAKE_CURRENT_SOURCE_DIR}/package.xml ...)
```

**Motivo:** la variante automática recorre también el árbol de build y reporta
los archivos generados por CMake como violaciones de estilo. Producía **192
fallos falsos**.

`epos4_bringup` y `epos4_interfaces` todavía usan `ament_lint_auto`; no
molesta porque no tienen fuentes C++.

---

## 15. Qué se arregló en esta sesión

Al ejecutar el build completo aparecieron **dos fallos reales** que antes
estaban enmascarados porque los paquetes se habían construido por separado.

### 15.1 `epos4_ros2_control` no compilaba: nombre de test duplicado

```
CMake Error at ament_add_test.cmake:125 (add_test):
  add_test given test NAME "xmllint" which already exists in this directory.
```

**Causa:** `ament_xmllint` nombra el test según sí mismo, así que **dos
llamadas en el mismo directorio colisionan**.

**Arreglo** en `src/epos4_ros2_control/CMakeLists.txt`:

```cmake
# One call, not one per file: ament_xmllint names the test after itself,
# so two calls in the same directory collide on the test name "xmllint".
ament_xmllint(
  ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
  ${CMAKE_CURRENT_SOURCE_DIR}/epos4_ros2_control_plugin.xml
)
```

> **Corrección honesta:** en la sesión anterior se dio por bueno que
> `epos4_ros2_control` "compilaba limpio". No era cierto para el build completo
> del workspace. Ahora sí lo es, verificado.

### 15.2 `epos4_interfaces/package.xml` no validaba contra el esquema

```
element test_depend: Schemas validity error : Element 'test_depend':
  This element is not expected. Expected is one of ( member_of_group, export ).
```

**Causa:** en `package format 3`, `<member_of_group>` va **después de todos los
`*_depend`** y antes de `<export>`. Estaba antes de los `test_depend`.

**Arreglo:** mover `<member_of_group>` debajo de los `test_depend`, con un
comentario explicando el orden.

### 15.3 Reformateo de `epos4_sim.cpp`

`ament_uncrustify --reformat` sobre `src/epos4_driver/tools/epos4_sim.cpp`
(11 líneas de divergencia, del código CSP añadido).

---

## 16. Trampas ya pisadas

Esta tabla es probablemente **lo más valioso del documento**. Cada fila costó
tiempo real de depuración. No volver a caer.

| Síntoma | Causa real | Arreglo |
|---|---|---|
| `es='J'` *Configuration download failed* | `UploadFile=` en `master.dcf` es **relativo**, y Lely lo resuelve **en el boot**, no en la carga | `CanBus` reescribe las rutas a absolutas en `master.resolved.dcf` |
| `es='D'` *identity mismatch* | `dcfgen` saca la identidad de `[DeviceInfo] VendorNumber` del EDS y el master la verifica contra `0x1018` | Red de simulación separada, con su propio EDS y DCF |
| `fiber_resume_with: Assertion 'curr' failed` | `FiberDriver` **debe construirse en el hilo del bus** | Cambiar a `LoopDriver` |
| El primer SDO falla: *"SDO connection not available"* | El master todavía no ha booteado el nodo | `WaitUntilReady()`, que **sondea un SDO explícitamente**. `IsReady()` no sirve: es falso ante un `es='L'` que es solo informativo |
| `bus.Stop()` provocaba un error de bus | `Stop()` destruía el master | `Stop()` ya no lo destruye |
| **Los PDOs nunca se activaban** | **Los dispositivos deben declararse ANTES de `bus.Start()`**: el master solo enruta PDOs a drivers registrados en el boot | Cambio de forma de la API. Es lo que fuerza el orden de `on_configure` (§9.6) |
| El master mandaba `controlword = 0` en cada SYNC, pisando lo escrito por SDO | Las escrituras salientes se estaban condicionando a `PdoActive()`, que solo sabe de PDOs **recibidos** | Intentar siempre PDO primero para las salidas |
| `60_rpm` se convertía en `5999` | Truncamiento en la conversión de unidades | Redondear al más cercano; test de 10 pasadas ida-y-vuelta que detecta la deriva |
| `HomingMethod` con valores mal | Los métodos de umbral de corriente `-1..-4` tenían los nombres **intercambiados**; faltaban los métodos 1, 2, 7 y 11 | Corregido contra §3.5.3 |
| `QuickStopOption` tenía valores inexistentes | El manual dice que el rango es **6 a 6**; había 0 y 2 | Reducido a un solo valor. `FaultReactionOption` además le faltaba el 1 |
| CSV salía por SDO en vez de PDO | El camino de PDO acababa en `ProfileVelocity` en vez de `CyclicVelocity`. **Dos comentarios apilados y contradictorios lo delataron** | Corregido. CSV existe *precisamente* para ir por PDO |
| `WithPosition(90_deg)` no compilaba | `degree_t → turn_t → Setpoint` son **dos** conversiones de usuario | Constructor templatizado con `is_convertible_unit_t` |
| Dos constructores SFINAE colisionaban | Los parámetros de plantilla **de tipo** con default se ignoran al comparar firmas | Distinguirlos con parámetros **no-tipo** de tipos distintos (`int` vs `bool`) |
| `std::is_convertible<ampere_t, degree_t>` da **`true`** | nholthaus valida con `static_assert` **dentro del constructor**, no con SFINAE. **El trait miente** | Documentado en un test. No construir SFINAE sobre ese trait |
| 192 fallos de lint falsos | `ament_lint_auto` recorría `build/` | Linters explícitamente acotados (§14) |
| `add_test NAME "xmllint" already exists` | `ament_xmllint` nombra el test según sí mismo | Una sola llamada con varios archivos (§15.1) |
| `package.xml` inválido | Orden de elementos del esquema format 3 | `<member_of_group>` después de los `*_depend` (§15.2) |
| El mock de `canopen_fake_slaves` no servía | Responde `0x0040` a **cualquier** escritura del Controlword y nunca avanza | Se escribió `epos4_sim` |
| PPM/CSP "no verificable" en simulación; el esclavo conserva el mapeo del EDS | **Diagnóstico equivocado.** `epos4_sim` (vendor `0xFB`) contra el DCF de `sim_network` (espera `0x555`): `es='D'` aborta el boot antes de descargar el mapeo | ✅ Usar `epos4_network` con el simulador — §10.3 |
| PPM: handshake perfecto pero el eje no se mueve (captura `0 -> 0`) | Target position (`0x607A`) se escribía solo por SDO; está en el RPDO1 con transmisión 1, así que el siguiente SYNC la pisaba con la copia del master (0) | `Impl::WriteOutput()`: toda salida mapeada va por RPDO, SDO solo si no está mapeada. Aplicado a los 5 modos |
| El simulador capturaba el target anterior aunque el RPDO traía el nuevo | Lely escribe los objetos de un RPDO uno a uno en orden de mapeo y llama a `OnWrite` tras cada uno; el Controlword va antes que `0x607A` | `epos4_sim` aplaza `HandleControlword()` con `GetExecutor().post()` |
| Segfault en `Epos4System::on_configure` → `Epos4::SetMechanism` | `Encoder` (que guarda la escala) vivía en `Impl`, creado en `Attach()` = dentro de `Start()`; el plugin fija la mecánica ANTES de `Start()`, como es obligatorio | `Configurator` y `Encoder` pasan a ser miembros de `Epos4`, creados en el constructor. Test: `test_device_lifecycle` |
| En CSP el simulador no se movía nunca; el JTC disparaba la tolerancia y "se quedaba" en 0 | `HandleCsp()` rearmaba el temporizador de 20 ms en cada SYNC de 10 ms: nunca expiraba | `ArmMotion()` no rearma si ya hay un paso pendiente (`motionArmed_`) |
| Frame `0F 00 00 00 00 00` (habilitado, objetivo 0) al final de una prueba | **No es un bug:** el JTC, tras violar la tolerancia, mantiene la posición *medida*, que era 0 porque el simulador no se movía | — |
| `read()` se bloqueaba al detectar un eje muerto | El log llamaba a `DescribeLastError()`: dos lecturas SDO contra un nodo mudo, cada una esperando el timeout, dentro del lazo RT | Log solo con datos en memoria: edad del PDO, Statusword cacheado, último EMCY **con su edad** (`GetCachedErrorCode`, `GetTimeSinceLastEmergency`) |
| Tras un error, el master seguía mandando «Enable operation» (`0x000F`) en cada SYNC a un eje sin supervisión | El plugin no implementaba `on_error`: ros2_control lo deja `unconfigured` sin pasar por `on_deactivate` | `on_error` → `StopAxes()`: `ExitCyclicMode` + `Disable`. El bus se mantiene para que siga saliendo `0x000D` |
| Posible UB al salir con Ctrl-C estando activo | `bus_` se declara después de `axes_` → se destruía antes que los dispositivos | Destructor + `Release()` (dispositivos primero). `on_configure` también libera lo que dejó `on_error` |
| `SetEmergencyCallback()` antes de `Start()` no hacía nada | `if (!impl_) return;` silencioso | Se guarda en `pendingEmcyCallback_` y se instala en `Attach()` |
| El log mostraba `last EMCY 0x8220` como causa | Era un EMCY del arranque del simulador (CiA 301, longitud de PDO), 20 s antes | Se imprime con su edad |
| (Teórico, ARM) primer SYNC tras `EnterCyclicPositionMode()` podía publicar objetivo 0 | Consigna sembrada y flag `cyclicActive` ambos `relaxed`: el hilo del bus podía ver el flag antes que la siembra | `CyclicState`: store release / load acquire del flag; test `AReaderThatSeesActiveSeesTheSeededTarget` |
| `QuickStop()`/`Halt()` en modo cíclico duraban un solo SYNC | `OnSync` republicaba la copia del Controlword tomada al entrar en modo cíclico | `WriteControlword()` actualiza también `cyclic.SetControlword()` |
| La trayectoria "no movía el eje" en una prueba | `ros2 topic pub --once` publicó antes del descubrimiento DDS; el driver estaba bien | `ros2 topic pub -w 1 --once` |
| `colcon test` fallaba con un código que ya estaba corregido; el binario de `build/` pasaba | CMake decide "Up-to-date" en el install comparando mtimes con resolución de 1 s: una `.so` recompilada en el mismo segundo que la anterior no se copió a `install/`, y los tests cargan la de `install/` | Comparar la sección `.text` (`objcopy -O binary --only-section=.text`), **no** el md5 del archivo: el install reescribe el RUNPATH de las `.so` que enlazan contra otras del build, y el md5 difiere aunque el código sea idéntico. Si `.text` difiere, borrar la de `install/` y recompilar |
| Imad no veía los archivos | Se estaba trabajando en un directorio temporal | Todo directo a `src/` |
| Se iba demasiado rápido en CANopen | — | Se volvió a cero: tramas CAN, arbitraje, COB-ID, tipos de mensaje, bus-off, epoll |

### Detalle de la API de Lely que cuesta encontrar

```cpp
// ✅ CORRECTO: convierte implícitamente al asignar
const std::int32_t value = rpdo_mapped[idx][0];

// ❌ NO EXISTE
const auto value = rpdo_mapped[idx][0].Read<std::int32_t>();
```

Y escribir en un objeto **no mapeado** en `tpdo_mapped` **lanza**. De ahí el
`try/catch` de `OnSync` (§8.5).

---

## 17. Mensajes de commit sugeridos

> **Recordatorio: estos los ejecuta Imad.** La sesión no debe correr `git add`
> ni `git commit`.

Hay bastante sin commitear (`git status` muestra 19 modificados y 7 sin
rastrear). Sugerencia: **cuatro commits temáticos** en vez de uno gigante.

### Commit 1 — unidades

```
feat(robot_units): vendor nholthaus/units v2.3.5 as a reusable package

A single-header, C++14 units library with compile-time dimensional
analysis and no runtime cost. Vendored rather than depended on: it is
not packaged for ROS Humble, and the team wants the same header
available across the whole robot without an apt that does not exist.

Driver code now expresses quantities instead of raw counts:

    motor.SetControl(controls::ProfilePosition{}.WithPosition(90_deg));

A test documents that std::is_convertible<ampere_t, degree_t> returns
true: nholthaus validates with a static_assert inside the constructor
rather than with SFINAE, so the trait cannot be built on.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
```

### Commit 2 — unidades en el driver

```
feat(epos4_driver): express control and feedback in physical units

MechanismScale converts between joint angles and encoder quadcounts
from the resolution and the gear ratio, and Setpoint<Raw, Quantity>
accepts either form so the raw values the manual and EPOS Studio use
still work.

Conversions round to nearest rather than truncating. Truncation lost a
count per conversion, which a ten-pass round-trip test caught: 60 rpm
came back as 5999.

Also corrects three enum transcription errors found while auditing the
tables against the manual:
  - HomingMethod: the current-threshold methods -1..-4 had their names
    swapped, and methods 1, 2, 7 and 11 were missing
  - QuickStopOption: carried two values the manual does not define;
    section 3.5.3 gives the range as 6 to 6
  - FaultReactionOption: was missing value 1

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
```

### Commit 3 — el camino de tiempo real

```
feat(epos4_driver): add a lock-free cyclic path for control loops

Every driver read and write, including the ones backed by PDO, went
through a promise, a Defer onto the bus thread and a blocking
future.get(). At 500 Hz with six axes that is thousands of context
switches per second in the control loop, and SetControl additionally
did one SDO read per command through EnsureMode.

That is fine for configuring and for moving to poses. It is not usable
from read()/write().

Inbound TPDOs now land in atomics in OnRpdoWrite, and staged setpoints
are published from OnSync on the bus thread, so the control loop only
ever does relaxed atomic loads and stores:

    EnterCyclicPositionMode()   sets the mode once, not per cycle
    GetCachedPosition/Velocity/Torque/Statusword
    StageTargetPosition()
    IsCyclicHealthy()

EnterCyclicPositionMode seeds the staged target with the current
position, or the first SYNC would command a move to the origin.

IsCyclicHealthy exists because cached values keep reading back happily
when the bus goes quiet, so a controller would go on believing a dead
axis is tracking.

The simulator learns CSP: it interpolates towards the received target
over the interpolation time period instead of running the PPM ramp,
and reports «Drive follows command value».

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
```

### Commit 4 — el plugin

```
feat(epos4_ros2_control): add the Epos4System hardware interface

A hardware_interface::SystemInterface owning one CAN bus and one Epos4
per joint. Commands go out as Cyclic Synchronous Position: the
trajectory is generated by the controller and the drive interpolates
between setpoints, which is the model joint_trajectory_controller
assumes. Profile Position generates the ramp inside the drive and does
not fit that model.

    command interfaces:  position
    state interfaces:    position, velocity, effort

Everything is in radians at the joint; MechanismScale converts, from
quadcounts_per_revolution and gear_ratio in the URDF.

read() and write() touch no bus. They are relaxed atomic loads and
stores against the cyclic path in epos4_driver.

Two things worth calling out:

  - on_configure constructs the devices BEFORE CanBus::Start(). A
    CANopen master boots each slave once and only routes a node's PDOs
    to a driver registered at that moment; constructing them afterwards
    gives nodes that answer SDO perfectly and never deliver a PDO.

  - on_activate seeds the command with the position already read, or
    the first write() sends the default of zero and the arm swings to
    its origin the instant the controller activates.

read() returns ERROR when IsCyclicHealthy() goes false, and logs it
once rather than once per axis per cycle.

12 tests cover on_init validation: a joint without a position command,
interfaces the plugin cannot provide, duplicate node IDs, a zero gear
ratio, a missing encoder resolution, and the radians-to-counts
round trip.

Also fixes two build failures that only appeared on a full-workspace
build: ament_xmllint names its test after itself so two calls in one
directory collide, and package format 3 requires <member_of_group>
after every *_depend.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
```

### Commit 5 — documentación

```
docs: add README, and a handoff document for the next session

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
```

---

## 18. Deuda técnica y backlog

### Bloqueantes para la verificación

| # | Cosa | Dónde |
|---|---|---|
| 1 | ~~El simulador no aplica el mapeo PDO~~ — era la red equivocada, resuelto | §10.3 |
| 2 | Faltan los tres archivos de bringup | §12 |
| 3 | Faltan los tests del camino cíclico | §11 paso 3 |

### Heartbeat consumer ✅ HECHO (2026-09-24)

El drive vigila al master: `heartbeat_consumer: true` en `epos4_network/bus.yml`
y heartbeat del master a 100 ms → `0x1016:01 = 0x0001012C` (nodo 1, 300 ms).
Verificado con `kill -9` de `ros2_control_node`: EMCY `0x8130` a los 300 ms,
el nodo pasa a pre-operacional (heartbeat `05` → `7F`) y deja de emitir PDOs.

✅ **`ClearFault()` ya recupera `0x8130`/`0x8120`** (§7.2.35): manda NMT reset
communication, espera al `OnBoot` del nodo (contador `bootCount` en `Impl`) y
luego hace el fault reset. Decide con `signals::RequiresCommunicationReset()`,
derivada del texto de recuperación transcrito. Verificado congelando el master
1 s con `SIGSTOP`: limpio en ~540 ms y re-habilitado con PDOs.

De paso: las lecturas "preferir PDO" solo usan la caché si el PDO tiene
< 100 ms (antes `ClearFault()` veía «Operation enabled» viejo y devolvía
`true` con el eje en Fault), y `epos4_sim` distingue reset node de reset
communication.

### Nada ha corrido contra hardware real

**Todo está verificado contra el simulador.** Es la limitación número uno y
está escrita en el CHANGELOG y el README.

Sin ejercitar todavía:
- **Homing (HMM)** — implementado, nunca ejecutado. Crítico: con encoders
  incrementales, **sin homing no hay cero absoluto**, y un brazo que no sabe
  dónde está no sirve.
- **CSV y CST** — implementados, nunca ejecutados.
- El freno de retención en un eje real.
- `Save()` a memoria no volátil.

### Del manual, sin envolver todavía

- Entradas analógicas y salidas analógicas
- Protección térmica
- Limitación de potencia
- Seguridad funcional (STO)
- Control de posición de doble lazo
- Observador de velocidad
- Touch probe
- Data recorder
- `Configurator::Refresh()` solo lee **un subconjunto** de la configuración

### Otros

- `epos4_interfaces` vacío (§6.5)
- El grupo de factores de unidades SI solo está hecho a medias
  (`SiUnitConfigs`)
- Confirmar el modelo del EPOS4 (§3)
- Limpiar `src/epos4_driver/install/` (§4)
- Actualizar `CHANGELOG.md`: sigue diciendo que `epos4_ros2_control` está vacío
- `README.md` no menciona el plugin y **está sin commitear**

### Ideas descartadas, con motivo

| Idea | Por qué no |
|---|---|
| Librería de unidades propia | Rechazada dos veces por Imad. Se vendoriza nholthaus |
| `ros2_canopen` / `canopen_402_driver` como dependencia | Útil solo como **lectura comparativa**. El encargo es librería propia |
| PPM bajo `ros2_control` | No encaja con el modelo cíclico (§7.10) |
| Interfaz de comando de velocidad (CSV) | Fuera de alcance por decisión de Imad. Añadirla luego es el mismo patrón más `prepare/perform_command_mode_switch` |
| Remapear PDOs en caliente desde la aplicación | Configuraría el drive dejando el diccionario del master obsoleto. El mapeo vive en `bus.yml` |

---

## 19. Referencias del manual

**EPOS4 Firmware Specification**, ed. 2026-07, rel13740, 310 páginas.
La página del visor = la página impresa.

| Tema | Sección | Pág. |
|---|---|---|
| Arquitectura | §2.1 | 13 |
| **Máquina de estados CiA 402** | §2.2 | 14–16 |
| Tabla de estados (Statusword) | Tabla 2-5 | 15 |
| **Tabla de comandos (Controlword)** | Tabla 2-7 | 16 |
| **PPM** | §3.3 | 21–24 |
| Handshake de setpoint PPM | Tabla 3-15 | 23 |
| PVM | §3.4 | 25–27 |
| **Homing** | §3.5 | 28–36 |
| Métodos de homing | §3.5.3 | 30–35 |
| **CSP** | §3.6 | 37–40 |
| CSV | §3.7 | 41–43 |
| CST | §3.8 | 44–46 |
| CAN | §5.2 | 61 |
| Tipos y acceso del OD | §6.1 | 63 |
| Heartbeat | §6.2.9 / §6.2.10 | 73–74 |
| Diagnosis history (`0x1003`) | §6.2.13 | 77 |
| **Mapeo PDO** | §6.2.15–6.2.30 | 79–110 |
| Node-ID (`0x2000`) | §6.2.40 | ~130 |
| CAN bit rate (`0x2001`) | §6.2.41 | ~131 |
| Abort connection option (`0x6007`) | §6.2.92 | 213 |
| **Controlword (`0x6040`)** | §6.2.94 | 215 |
| **Statusword (`0x6041`)** | §6.2.95 | 216 |
| Modes of operation (`0x6060`) | §6.2.101 | 220 |
| Halt option code (`0x605D`) | Tabla 6-152 | ~218 |
| **EMCY** | §7.1 | 265 |
| **Device errors** | §7.2 | 265–284 |
| Tabla de device errors | Tabla 7-186 | 283 |
| **Abort codes SDO** | §7.3 / Tabla 7-187 | 285 |

**EPOS4 Communication Guide**, ed. 2026-04, rel13604: framing CANopen y
asignación de COB-ID.

Extraer una sección:

```bash
pdftotext -f 215 -l 216 -layout \
  resources/EPOS4-Firmware-Specification-En-1.pdf -
```

---

## 20. Glosario

| Término | Qué es |
|---|---|
| **CiA 301** | El estándar CANopen base: diccionario de objetos, SDO, PDO, NMT, SYNC, EMCY |
| **CiA 402** | El perfil de drives eléctricos encima de CiA 301: la máquina de estados y los modos de operación |
| **COB-ID** | El identificador de la trama CAN = código de función + node-ID |
| **CPR** | Counts Per Revolution del encoder. **Quadcounts = CPR × 4** |
| **CSP / CSV / CST** | Cyclic Synchronous Position / Velocity / Torque. Modos 8 / 9 / 10 |
| **DCF** | Device Configuration File. El EDS **con los valores concretos de esta red** |
| **`dcfgen`** | La herramienta de Lely que convierte `bus.yml` + EDS en `master.dcf` y `node_N.bin` |
| **EDS** | Electronic Data Sheet. Describe el diccionario de objetos de un dispositivo |
| **EMCY** | Emergency. El nodo grita un fallo sin que nadie pregunte |
| **HMM** | Homing Mode, modo 6 |
| **NMT** | Network Management. Start / Stop / Pre-operational / Reset |
| **PDO** | Process Data Object. Rápido, sin confirmar, **premapeado**. RPDO = master→nodo, TPDO = nodo→master |
| **PPM** | Profile Position Mode, modo 1. La rampa la genera el drive |
| **PVM** | Profile Velocity Mode, modo 3 |
| **Quadcount** | Una cuenta de cuadratura. Cuatro por pulso del encoder |
| **SDO** | Service Data Object. Fiable, con respuesta, cualquier objeto, lento |
| **SYNC** | Pulso que difunde el master; los nodos aplican su setpoint en ese instante |
| **Bus-off** | Un nodo CAN se desconecta solo tras demasiados errores de transmisión |
| **`vcan0`** | Interfaz CAN virtual de Linux, para desarrollo sin hardware |

---

## Resumen en una pantalla

```
DÓNDE ESTAMOS
  Driver EPOS4 en C++ sobre Lely, estilo API de CTRE Phoenix.
  5 paquetes, 184 tests en verde, 0 fallos.
  Partes A (camino de tiempo real) y B (plugin ros2_control): HECHAS.
  NADA ha corrido contra hardware real.

QUÉ FALTA, EN ORDEN
  1. ✅ Mapeo PDO del simulador: no había bloqueo (red equivocada, §10.3)
  2. ✅ Bringup (xacro, urdf, controllers.yaml, launch.py) — verificado
  3. ✅ Tests del camino cíclico (core::CyclicState, 15 tests)
  4. ✅ End-to-end verificado, incluida la red de seguridad (§13.7)
  5. ✅ Limpieza, CHANGELOG y README

REGLAS QUE NO SE ROMPEN
  · Git lo ejecuta Imad. La sesión solo redacta el mensaje del commit.
  · Escribir en src/, nunca en un directorio temporal.
  · Código y comentarios en inglés; conversación en español.
  · Ninguna dirección hexadecimal desnuda fuera de ObjectDictionary.hpp.
  · epos4_driver no incluye ni un header de ROS.
  · Escribir el código completo, explicando. La fase de tutoría terminó.
```

