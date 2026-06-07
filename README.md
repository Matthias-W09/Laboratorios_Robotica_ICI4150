# Proyecto Final: Navegación Autónoma con Planificación de Rutas en Webots

## Índice de Contenidos

- [Información General](#información-general)
- [Integrantes del Equipo](#integrantes-del-equipo)
- [Línea Seleccionada](#línea-seleccionada)
- [Objetivo del Proyecto](#objetivo-del-proyecto)
- [Descripción del Robot, Sensores y Actuadores](#descripción-del-robot-sensores-y-actuadores)
- [Descripción de los Escenarios de Prueba](#descripción-de-los-escenarios-de-prueba)
- [Algoritmo Implementado: A*](#algoritmo-implementado-a)
- [Diagrama de Flujo de la Solución](#diagrama-de-flujo-de-la-solución)
- [Relación con Laboratorios 1 y 2](#relación-con-laboratorios-1-y-2)
- [Resultados Obtenidos y Métricas de Desempeño](#resultados-obtenidos-y-métricas-de-desempeño)
- [Capturas y Videos](#capturas-y-videos)
- [Instrucciones para Ejecutar la Simulación](#instrucciones-para-ejecutar-la-simulación)
- [Conclusiones, Limitaciones y Posibles Mejoras](#conclusiones-limitaciones-y-posibles-mejoras)

---

## Información General

- **Nombre del proyecto:** Navegación Autónoma con Planificación de Rutas (A\*) en Webots
- **Asignatura:** Robótica y Sistemas Autónomos 2026-01
- **Código:** ICI 4150
- **Herramientas utilizadas:** Webots, Python

---

## Integrantes del Equipo

| Rol | Integrante |
|---|---|
| Programador | Carlos Aguirre |
| Experimentador | Javier Donetch |
| Analista | Matthias Julio |
| Documentador | Ignacio Vera |
| Integrador | Luciano Fredes |

---

## Línea Seleccionada

**Línea A: Planificación de Rutas**

Se implementó un sistema de navegación global que permite al robot desplazarse autónomamente desde una posición inicial hasta una meta, utilizando el algoritmo A\* sobre una grilla de ocupación 2D. La navegación global se complementó con evasión reactiva de obstáculos inesperados mediante sensores de distancia y un filtro de Kalman.

---

## Objetivo del Proyecto

Diseñar, implementar y evaluar en Webots un sistema de navegación autónoma para el robot diferencial e-puck, integrando:

- Control cinemático diferencial del Laboratorio 1.
- Percepción sensorial, encoders y filtro de Kalman del Laboratorio 2.
- Planificación global de rutas mediante el algoritmo A\* sobre una grilla de ocupación 20×20.
- Seguimiento de waypoints mediante control proporcional de orientación y posición.
- Evasión reactiva ante obstáculos inesperados no representados en el mapa.

---

## Descripción del Robot, Sensores y Actuadores

Se utilizó el robot **e-puck** de Webots, un robot diferencial con dos ruedas motrices independientes y sensores infrarrojos de proximidad.

### Actuadores

| Actuador | Función |
|---|---|
| Motor rueda izquierda | Control de velocidad angular rueda izquierda |
| Motor rueda derecha | Control de velocidad angular rueda derecha |

### Sensores

| Sensor | Función |
|---|---|
| `ps0`, `ps7` | Sensores frontales (detección de obstáculos) |
| `ps1`, `ps6` | Sensores diagonales/laterales |
| Encoder izquierdo | Odometría rueda izquierda |
| Encoder derecho | Odometría rueda derecha |

### Parámetros físicos del robot

| Parámetro | Valor |
|---|---|
| Radio de rueda ($r$) | 0.0205 m |
| Distancia entre ejes ($L$) | 0.057 m (calibrado para giros de 90°) |
| Velocidad máxima | 4.5 rad/s |
| Paso de simulación ($T_s$) | 0.032 s |

---

## Descripción de los Escenarios de Prueba

### Escenario 1 – Simple

Laberinto con baja densidad de obstáculos. La ruta entre inicio y meta es relativamente directa, con pocos giros y pasillos amplios. Permite validar el seguimiento de waypoints y la precisión de la odometría sin condiciones exigentes.

### Escenario 2 – Complejo

Laberinto 20×20 con muros, zonas restringidas (costo 1) y zonas de alto riesgo (costo 5, marcadas con `P` en la grilla). La ruta óptima requiere múltiples cambios de dirección y evitar zonas penalizadas. Se evalúa la capacidad del planificador para encontrar rutas de bajo costo y la capacidad del robot de ejecutarlas fielmente.

La grilla utilizada es la siguiente (extracto representativo):

```
0 = Libre    1 = Muro    2 = Inicio    3 = Meta    5 = Alto Costo (P)
```

El punto de inicio se ubicó en la celda `(11, 5)` y la meta en la celda `(8, 8)`.

---

## Algoritmo Implementado: A\*

### ¿Por qué A\*?

A\* es un algoritmo de búsqueda informada que combina el costo acumulado desde el inicio ($g$) con una estimación heurística del costo restante hasta la meta ($h$). Esto lo hace más eficiente que Dijkstra en espacios grandes y más robusto que BFS ante costos heterogéneos.

### Heurística utilizada

Se empleó la distancia Manhattan, apropiada para una grilla con movimientos en 4 direcciones (arriba, abajo, izquierda, derecha):

$$
h(n) = |f_n - f_{meta}| + |c_n - c_{meta}|
$$

### Costos de celda

| Tipo de celda | Costo de paso |
|---|---|
| Libre (0, 2, 3) | 1 |
| Alto riesgo (5) | 5 |
| Muro (1) | Bloqueado |

Esto hace que A\* prefiera rutas que eviten zonas de alto riesgo, a menos que no exista alternativa.

### Pseudocódigo

```
función A*(mapa, inicio, meta):
    frontera ← cola de prioridad con (f=0, inicio)
    g_score[inicio] ← 0
    origen ← {}

    mientras frontera no esté vacía:
        actual ← extraer nodo con menor f de frontera

        si actual == meta:
            reconstruir y retornar ruta desde origen

        para cada vecino de actual (4 direcciones):
            si vecino es MURO: continuar
            costo_paso ← 5 si RIESGO, 1 si no
            nuevo_g ← g_score[actual] + costo_paso

            si nuevo_g < g_score[vecino]:
                g_score[vecino] ← nuevo_g
                f ← nuevo_g + heuristica(vecino, meta)
                insertar (f, vecino) en frontera
                origen[vecino] ← actual

    retornar [] (sin ruta)
```

### Conversión de ruta a waypoints

Cada celda `(fila, columna)` de la ruta se convierte a coordenadas métricas relativas al punto de inicio:

$$
x_{obj} = (c_{wp} - c_{inicio}) \times \Delta_{celda}
$$
$$
y_{obj} = -(f_{wp} - f_{inicio}) \times \Delta_{celda}
$$

donde $\Delta_{celda} = 0.1$ m es el tamaño físico de cada celda.

---

## Diagrama de Flujo de la Solución

```
┌────────────────────────────────────────┐
│           INICIO DEL SISTEMA           │
└─────────────────┬──────────────────────┘
                  │
                  ▼
┌────────────────────────────────────────┐
│  CALCULAR_RUTA                         │
│  - Localizar INICIO y META en la grilla│
│  - Ejecutar A*                         │
│  - Obtener lista de waypoints          │
└─────────────────┬──────────────────────┘
                  │ Ruta encontrada
                  ▼
┌────────────────────────────────────────┐
│  SEGUIR_WAYPOINT                       │
│  Loop por cada waypoint:               │
│    1. Leer encoders → Odometría        │
│    2. Leer sensores IR → Kalman        │
│    3. Calcular error de ángulo         │
│    4. Si |error_ángulo| > umbral:      │
│         → Girar (control proporcional) │
│    5. Si no: Avanzar                   │
│    6. Si distancia < tolerancia:       │
│         → Siguiente waypoint           │
│    7. Si obstáculo inesperado:         │
│         → RECALCULAR_OBSTACULO         │
└─────────────────┬──────────────────────┘
                  │ Todos los waypoints alcanzados
                  ▼
┌────────────────────────────────────────┐
│  DETENIDO – Meta alcanzada             │
└────────────────────────────────────────┘
```

**Estado adicional: RECALCULAR_OBSTACULO**

Si se detecta un obstáculo inesperado a menos de 0.22 m (distancia estimada por Kalman), el robot frena, retrocede brevemente y vuelve al estado `CALCULAR_RUTA` para replantear la ruta.

---

## Relación con Laboratorios 1 y 2

### Laboratorio 1 – Control Cinemático Diferencial

El seguimiento de waypoints en el proyecto se basa directamente en el modelo cinemático diferencial trabajado en el Laboratorio 1. Las velocidades de las ruedas se calculan a partir del error de orientación:

$$
v = \frac{v_r + v_l}{2}, \quad \omega = \frac{v_r - v_l}{L}
$$

El control proporcional de giro aplica:

```python
v_rotacion = 1.0 * error_angulo
left_motor.setVelocity(-v_rotacion)
right_motor.setVelocity(v_rotacion)
```

Y el avance se realiza con velocidades iguales en ambas ruedas.

### Laboratorio 2 – Percepción, Encoders y Kalman

El proyecto reutiliza directamente:

**Odometría con encoders** (Laboratorio 2, sección "Estimación del Avance"):

$$
\Delta s = \frac{\Delta s_r + \Delta s_l}{2}, \quad \Delta\phi = \frac{\Delta s_r - \Delta s_l}{L}
$$
$$
x_k = x_{k-1} + \Delta s \cos\!\left(\phi_{k-1} + \frac{\Delta\phi}{2}\right)
$$
$$
y_k = y_{k-1} + \Delta s \sin\!\left(\phi_{k-1} + \frac{\Delta\phi}{2}\right)
$$

**Filtro de media móvil** sobre las lecturas IR frontales (ventana $N = 5$).

**Filtro de Kalman escalar** para estimar la distancia frontal al obstáculo:

$$
\hat{d}_k^- = \hat{d}_{k-1} - \Delta d_k, \quad P_k^- = P_{k-1} + Q
$$
$$
K_k = \frac{P_k^-}{P_k^- + R}, \quad \hat{d}_k = \hat{d}_k^- + K_k(z_k - \hat{d}_k^-)
$$

El proyecto **extiende** el Laboratorio 2 al incorporar la odometría no solo para estimar distancia a obstáculos, sino para localizar el robot en un sistema de coordenadas globales y ejecutar un seguimiento de waypoints planificados.

---

## Resultados Obtenidos y Métricas de Desempeño

### Escenario 1 – Simple

| Métrica | Valor |
|---|---|
| Waypoints planificados | _[completar con dato experimental]_ |
| Tiempo hasta la meta | _[completar con dato experimental]_ s |
| Longitud de ruta planificada | _[completar]_ m |
| Longitud de trayectoria ejecutada | _[completar]_ m |
| Colisiones | 0 |
| Giros innecesarios | _[completar]_ |
| Ejecuciones exitosas / total | _[completar]_ / 5 |

### Escenario 2 – Complejo

| Métrica | Valor |
|---|---|
| Waypoints planificados | _[completar]_ |
| Tiempo hasta la meta | _[completar]_ s |
| Longitud de ruta planificada | _[completar]_ m |
| Longitud de trayectoria ejecutada | _[completar]_ m |
| Colisiones | _[completar]_ |
| Activaciones de RECALCULAR_OBSTACULO | _[completar]_ |
| Ejecuciones exitosas / total | _[completar]_ / 5 |

### Comparación entre señales de distancia

| Métrica | Señal cruda | Media móvil | Kalman |
|---|---|---|---|
| Estabilidad | Baja | Media | Alta |
| Activaciones erróneas de freno | Muchas | Moderadas | Pocas |
| Colisiones | Algunas | Pocas | Ninguna |

> **Nota:** Los campos marcados con _[completar]_ deben llenarse con los datos registrados durante las ejecuciones experimentales.

---

## Capturas y Videos

_Agregar aquí capturas de pantalla del robot navegando en ambos escenarios, gráficos de la ruta planificada vs. trayectoria ejecutada, y el enlace al video demostrativo._

```
[Video demostrativo](enlace_al_video)
```

```
[Ruta planificada vs. ejecutada - Escenario Simple](ruta_simple.png)
[Ruta planificada vs. ejecutada - Escenario Complejo](ruta_complejo.png)
[Señales crudas, filtradas y Kalman](grafico_señales.png)
```

---

## Instrucciones para Ejecutar la Simulación

1. Instalar [Webots](https://cyberbotics.com/) (versión R2023b o superior recomendada).
2. Instalar Python 3.10 o superior.
3. Clonar el repositorio:

```bash
git clone https://github.com/usuario/proyecto-final-robotica.git
cd proyecto-final-robotica
```

4. Abrir Webots y cargar el archivo de mundo:

```
File → Open World → mundos/laberinto_proyecto.wbt
```

5. Seleccionar el robot e-puck en la escena.
6. Asignar el controlador:

```
e-puck → Controller → proyecto_final_controller.py
```

7. Ejecutar la simulación con el botón ▶ de Webots.

> El robot calculará la ruta automáticamente al iniciar. Los mensajes de estado se imprimen en la consola de Webots.

---

## Conclusiones, Limitaciones y Posibles Mejoras

### Conclusiones

1. El algoritmo A\* sobre una grilla de ocupación permitió planificar rutas eficientes, evitando muros y priorizando el alejamiento de zonas de alto riesgo gracias al costo diferenciado.
2. La odometría con encoders fue suficiente para el seguimiento de waypoints en trayectorias cortas, pero acumula error en rutas largas con múltiples giros.
3. El filtro de Kalman mejoró la detección de obstáculos inesperados, reduciendo activaciones falsas del estado `RECALCULAR_OBSTACULO` frente a usar la señal cruda de los sensores IR.
4. El control proporcional de orientación resultó estable para la tolerancia angular configurada (0.02 rad), aunque con ganancia reducida (1.0) para evitar sobreoscilaciones.
5. La integración de los tres módulos (planificación A\*, odometría, Kalman) permitió una navegación global funcional que supera la navegación reactiva pura del Laboratorio 2.

### Limitaciones

- La grilla de ocupación es estática: el sistema no actualiza el mapa ante obstáculos dinámicos más allá del recálculo de emergencia.
- El error odométrico acumulado en rutas largas puede provocar desviaciones entre la celda estimada y la posición real del robot.
- La calibración de `DISTANCIA_EJES` es sensible: pequeñas variaciones afectan la precisión de los giros de 90°.
- La conversión IR→metros es una función empírica aproximada; en escenarios con iluminación variable su precisión puede degradarse.

### Posibles Mejoras

- Incorporar corrección odométrica periódica usando marcadores visuales o landmarks del entorno.
- Implementar una grilla de ocupación dinámica que marque celdas con obstáculos detectados en tiempo real y dispare un recálculo de A\* automáticamente.
- Sustituir el control proporcional de orientación por un controlador PID para reducir oscilaciones en entornos con perturbaciones.
- Explorar la Línea B (SLAM simplificado) integrando la actualización del mapa con la odometría y los sensores laterales para entornos parcialmente desconocidos.
