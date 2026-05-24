# Laboratorio 2: Navegación Reactiva con Filtrado y Fusión de Sensores en Webots

## Índice de Contenidos

- [Información General](#información-general)
- [Integrantes del Equipo](#integrantes-del-equipo)
- [Objetivo](#objetivo)
- [Descripción del Robot y Sensores](#descripción-del-robot-y-sensores)
- [Frecuencia de Muestreo](#frecuencia-de-muestreo)
- [Análisis de Señales Registradas](#análisis-de-señales-registradas)
- [Estimación del Avance mediante Encoders](#estimación-del-avance-mediante-encoders)
- [Filtro Simple Aplicado](#filtro-simple-aplicado)
- [Filtro de Kalman](#filtro-de-kalman)
- [Lógica de Navegación Reactiva](#lógica-de-navegación-reactiva)
- [Instrucciones de Ejecución](#instrucciones-de-ejecución)
- [Resultados y Experimentos](#resultados-y-experimentos)
- [Evidencia Visual](#evidencia-visual)
- [Análisis Final y Conclusiones](#análisis-final-y-conclusiones)


## Información General
* **Asignatura:** Robótica y Sistemas Autónomos 2026-01
* **Código:** ICI 4150
* **Herramientas:** Webots, Python

## Integrantes del Equipo
* **Programador:** Carlos Aguirre [Paralelo 2] - Implementación del controlador
* **Experimentador:** Javier Donetch [Paralelo 2] - Ejecución de pruebas
* **Analista:** Matthias Julio [Paralelo 1] - Interpretación de resultados
* **Documentador:** Ignacio Vera [Paralelo 1] - Redacción del informe / Readme
* **Integrador:** Luciano Fredes [Paralelo 2] - Coordinación del trabajo

## Objetivo
Implementar un sistema básico de navegación reactiva en Webots para un robot móvil diferencial, utilizando sensores de distancia y encoders de rueda, aplicando filtrado sobre las mediciones y empleando un filtro de Kalman para estimar la distancia frontal a obstáculos y mejorar la toma de decisiones.

## Descripción del Robot y Sensores

Se utilizó el robot **e-puck** de Webots, un robot móvil diferencial con dos ruedas motrices independientes.

### Sensores utilizados
| Sensor | Descripción |
| :--- | :--- |
| `ps0`, `ps7` | Sensores frontales de distancia (izquierdo y derecho) |
| `ps5` | Sensor lateral izquierdo |
| `ps2` | Sensor lateral derecho |
| Encoder rueda izquierda | Medición angular del giro de la rueda izquierda |
| Encoder rueda derecha | Medición angular del giro de la rueda derecha |

Los sensores de distancia/proximidad entregan valores proporcionales a la cercanía de obstáculos, mientras que los encoders registran el desplazamiento angular acumulado de cada rueda.

## Frecuencia de Muestreo

El controlador se ejecuta con un paso de simulación fijo, definiendo los siguientes parámetros:

| Parámetro | Valor |
| :--- | :--- |
| Tiempo de muestreo ($T_s$) | `0.064 s` (64 ms) |
| Frecuencia de muestreo ($f_s = 1/T_s$) | `~15.6 Hz` |
| Muestras registradas por experimento | Variable según duración |

```python
TIME_STEP = 64  # ms
```

Todas las señales registradas, filtradas y estimadas fueron analizadas bajo esta misma frecuencia de muestreo.

## Análisis de Señales Registradas

Durante la simulación se registraron en tiempo real las lecturas crudas de los sensores de distancia y de los encoders.

### Señales de sensores de distancia (crudas)
Las lecturas de los sensores frontales presentan ruido significativo, con variaciones aleatorias incluso cuando el robot se encuentra estático o en movimiento uniforme. Esto motiva la aplicación de filtrado antes de utilizarlas para la toma de decisiones.

### Señales de encoders
Los encoders entregan valores angulares acumulados en radianes. Su comportamiento es más estable que los sensores de distancia, aunque también acumulan error a lo largo del tiempo.

Los valores registrados se almacenaron en listas para su posterior análisis y graficación:

```python
raw_front_log = []
filtered_front_log = []
kalman_front_log = []
encoder_left_log = []
encoder_right_log = []

# En cada iteración:
raw_front_log.append(raw_front)
encoder_left_log.append(left_encoder.getValue())
encoder_right_log.append(right_encoder.getValue())
```

## Estimación del Avance mediante Encoders

Los encoders del e-puck entregan medidas angulares en radianes. Para convertir esta información en desplazamiento lineal se utiliza la relación:

$$s = r\theta$$

Donde:
* **$s$**: desplazamiento lineal de la rueda
* **$r$**: radio de la rueda del e-puck (`0.0205 m`)
* **$\theta$**: variación angular medida por el encoder entre dos instantes consecutivos

El avance estimado del robot en cada paso se calcula como el promedio del desplazamiento de ambas ruedas:

$$\Delta d_k = \frac{s_{\text{izq}} + s_{\text{der}}}{2} = \frac{r(\Delta\theta_{\text{izq}} + \Delta\theta_{\text{der}})}{2}$$

```python
WHEEL_RADIUS = 0.0205  # metros

delta_left = (left_encoder.getValue() - prev_left_enc) * WHEEL_RADIUS
delta_right = (right_encoder.getValue() - prev_right_enc) * WHEEL_RADIUS
delta_d = (delta_left + delta_right) / 2.0

prev_left_enc = left_encoder.getValue()
prev_right_enc = right_encoder.getValue()
```

Este valor $\Delta d_k$ representa cuánto avanzó el robot entre dos instantes consecutivos y es utilizado como entrada en la etapa de predicción del filtro de Kalman.

## Filtro Simple Aplicado

Antes de implementar el filtro de Kalman, se aplicó un **filtro de media móvil** sobre las lecturas crudas de los sensores frontales. Este filtro promedia las últimas $N$ muestras para suavizar el ruido:

$$\hat{z}_k = \frac{1}{N} \sum_{i=0}^{N-1} z_{k-i}$$

```python
FILTER_WINDOW = 5  # número de muestras

sensor_buffer.append(raw_front)
if len(sensor_buffer) > FILTER_WINDOW:
    sensor_buffer.pop(0)

filtered_front = sum(sensor_buffer) / len(sensor_buffer)
```

### Comparación señal cruda vs. filtrada
Al graficar ambas señales, se observa que el filtro de media móvil reduce notablemente las fluctuaciones de alta frecuencia, entregando una señal más estable sin introducir un retardo excesivo.

## Filtro de Kalman

Se implementó un filtro de Kalman escalar para estimar la distancia frontal al obstáculo más cercano ($d_k$), combinando la predicción por encoders con la medición de los sensores frontales.

### Variable de estado
$$d_k = \text{distancia frontal estimada al obstáculo en el instante } k$$

### Etapa de Predicción

La distancia frontal se predice restando el avance estimado del robot:

$$\hat{d}_k^- = \hat{d}_{k-1} - \Delta d_k$$

La covarianza de predicción se actualiza sumando la incertidumbre del proceso:

$$P_k^- = P_{k-1} + Q$$

```python
d_pred = d_est - delta_d
P_pred = P_est + Q
```

### Etapa de Corrección

La corrección se realiza con la medición del sensor frontal filtrado ($z_k$):

$$K_k = \frac{P_k^-}{P_k^- + R}$$

$$\hat{d}_k = \hat{d}_k^- + K_k(z_k - \hat{d}_k^-)$$

$$P_k = (1 - K_k) \cdot P_k^-$$

```python
K = P_pred / (P_pred + R)
d_est = d_pred + K * (filtered_front - d_pred)
P_est = (1 - K) * P_pred
```

### Parámetros del filtro

| Parámetro | Descripción | Valor usado |
| :--- | :--- | :--- |
| $Q$ | Varianza del proceso (incertidumbre del modelo) | `0.01` |
| $R$ | Varianza de la medición (ruido del sensor) | `0.1` |
| $P_0$ | Covarianza inicial | `1.0` |
| $\hat{d}_0$ | Distancia inicial estimada | `1.0` |

La ganancia $K_k$ se recalcula en cada iteración:
- Si $R$ es grande (sensor ruidoso), $K_k$ disminuye → el filtro confía más en la predicción.
- Si $P_k^-$ es grande (modelo incierto), $K_k$ aumenta → el filtro confía más en la medición.

## Lógica de Navegación Reactiva

La toma de decisiones del robot se basa en la distancia frontal estimada por el filtro de Kalman y en las lecturas de los sensores laterales.

```python
UMBRAL_FRENTE = 0.15  # metros (umbral de seguridad)

if d_est > UMBRAL_FRENTE:
    # Avanzar
    left_motor.setVelocity(MAX_SPEED)
    right_motor.setVelocity(MAX_SPEED)
else:
    # Decidir dirección de giro según sensores laterales
    if sensor_izq.getValue() > sensor_der.getValue():
        # Obstáculo más cercano a la izquierda → girar a la derecha
        left_motor.setVelocity(MAX_SPEED)
        right_motor.setVelocity(-MAX_SPEED)
    else:
        # Obstáculo más cercano a la derecha → girar a la izquierda
        left_motor.setVelocity(-MAX_SPEED)
        right_motor.setVelocity(MAX_SPEED)
```

### Resumen de reglas de decisión

| Condición | Acción |
| :--- | :--- |
| $\hat{d}_k > \text{umbral}$ | Avanzar |
| $\hat{d}_k \leq \text{umbral}$ y sensor izq > sensor der | Girar a la derecha |
| $\hat{d}_k \leq \text{umbral}$ y sensor der ≥ sensor izq | Girar a la izquierda |

## Instrucciones de Ejecución

1. Descargar e instalar **Webots** desde su sitio oficial:
   [Descargar Webots](https://www.cyberbotics.com/#download)

2. Descargar e instalar **Python** desde el sitio oficial:
   [Descargar Python](https://www.python.org/downloads/)
   - Se utilizó la versión **Python 3.12.4**.

3. Descargar o clonar este repositorio en su computador.

4. En Webots, abrir el archivo de mundo incluido en el repositorio (`.wbt`) correspondiente al escenario deseado.

5. Seleccionar el robot dentro del entorno de simulación.

6. En las propiedades del robot:
   - Ubicar el campo **controller**
   - Asignar el archivo `lab2controller_epuck.py` incluido en este repositorio.

7. Presionar el botón **Play** en Webots para iniciar la simulación.

> **Nota:** Al finalizar la simulación, el controlador genera automáticamente los gráficos de las señales registradas (crudas, filtradas y estimadas con Kalman).

## Resultados y Experimentos

Se diseñaron dos escenarios de prueba en Webots:

### Escenario 1: Entorno simple
Un espacio abierto con pocos obstáculos distribuidos. El robot debía detectarlos y esquivarlos sin colisionar.

| Métrica | Con señal cruda | Con filtro simple | Con fusión Kalman |
| :--- | :--- | :--- | :--- |
| Estabilidad del movimiento | Baja (oscilaciones) | Media | Alta |
| Giros innecesarios | Frecuentes | Moderados | Escasos |
| Colisiones | Ocasionales | Pocas | Ninguna |

### Escenario 2: Entorno complejo
Pasillos estrechos y múltiples obstáculos. Se evaluó la capacidad de navegación en espacios reducidos.

| Métrica | Con señal cruda | Con filtro simple | Con fusión Kalman |
| :--- | :--- | :--- | :--- |
| Estabilidad del movimiento | Muy baja | Media | Alta |
| Giros innecesarios | Muy frecuentes | Moderados | Pocos |
| Colisiones | Frecuentes | Ocasionales | Ninguna |

### Comparación de señales
Al graficar las tres señales (cruda, filtrada y estimada con Kalman), se observa que:
- La señal **cruda** presenta ruido considerable.
- El **filtro simple** suaviza el ruido pero introduce un leve retardo.
- La **estimación con Kalman** produce la señal más estable y con menor retardo, al combinar la predicción cinemática con la medición del sensor.

## Evidencia Visual
A continuación se muestra la ejecución del controlador del robot en Webots.

### Demostración del robot
![Robot en funcionamiento](Robot-Video.gif)

### Gráficos de señales
![Señales registradas](graficos_señales.png)

### Descripción del comportamiento observado

- Avance recto cuando la distancia frontal estimada supera el umbral de seguridad.
- Giro reactivo al detectar obstáculos, con dirección determinada por los sensores laterales.
- Mayor estabilidad en las decisiones al usar la estimación de Kalman frente a las lecturas crudas.
- Reducción de giros innecesarios con la fusión sensorial en comparación a la señal sin filtrar.

## Análisis Final y Conclusiones

#### 1. **¿Qué diferencia se observa entre usar señales crudas, filtradas y estimadas con Kalman?**
Las señales crudas presentan ruido que genera decisiones inestables, provocando giros innecesarios. El filtro simple reduce este problema, pero introduce un retardo proporcional al tamaño de la ventana. El filtro de Kalman logra el mejor equilibrio, combinando la información de movimiento (encoders) con la medición del sensor, produciendo estimaciones más robustas con menor retardo.

#### 2. **¿Cómo funciona la predicción del filtro de Kalman en este contexto?**
La predicción utiliza el avance estimado del robot mediante encoders ($s = r\theta$) para anticipar cuánto se redujo la distancia frontal al obstáculo. Esto permite que el filtro mantenga una estimación razonable incluso cuando las lecturas del sensor son temporalmente ruidosas o poco confiables.

#### 3. **¿Qué ocurre cuando la ganancia de Kalman es alta o baja?**
Cuando el sensor es ruidoso ($R$ grande), la ganancia $K_k$ disminuye y el filtro confía más en la predicción cinemática. Cuando la incertidumbre del modelo es alta ($P_k^-$ grande), la ganancia aumenta y se otorga más peso a la medición del sensor. Este balance automático es la principal ventaja del filtro de Kalman frente a un filtro de ganancia fija.

#### 4. **¿Qué tan efectiva fue la navegación reactiva implementada?**
La lógica de navegación reactiva basada en la distancia estimada con Kalman demostró ser efectiva para evitar colisiones en ambos escenarios. La incorporación de los sensores laterales permitió elegir correctamente la dirección de giro, reduciendo la probabilidad de quedar atrapado frente a un obstáculo.

### Resumen de Conclusiones
1. **Filtrado:** El filtro de media móvil es simple y efectivo para reducir ruido, pero el filtro de Kalman supera su desempeño al incorporar un modelo de movimiento.
2. **Fusión sensorial:** La combinación de encoders y sensores de distancia mediante Kalman entrega estimaciones más confiables que cualquiera de las fuentes por separado.
3. **Navegación reactiva:** La toma de decisiones basada en distancia estimada reduce considerablemente los giros innecesarios y las colisiones respecto al uso de señales crudas.
4. **Modelo cinemático:** La relación $s = r\theta$ permite una estimación adecuada del avance del robot, sirviendo como predictor del estado en el filtro de Kalman.
