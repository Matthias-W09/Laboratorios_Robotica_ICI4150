
# Laboratorio 1: Simulación de un Robot Móvil Diferencial en Webots

## Índice de Contenidos

- [Información General](#información-general)
- [Integrantes del Equipo](#integrantes-del-equipo)
- [Descripción del Laboratorio](#descripción-del-laboratorio)
- [Modelo Cinemático](#modelo-cinemático)
- [Instrucciones de Ejecución](#instrucciones-de-ejecución)
- [Resultados y Experimentos](#resultados-y-experimentos)
- [Evidencia Visual](#evidencia-visual)
- [Análisis de Resultados](#análisis-de-resultados)
- [Conclusiones](#conclusiones)


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

## Descripción del Laboratorio
El objetivo de esta actividad es comprender y analizar el comportamiento cinemático de un robot móvil diferencial (robot diferencial tipo e-puck) mediante simulaciones interactivas. 
En este modelo, el movimiento del robot es determinado por el control independiente de las velocidades de sus dos ruedas motrices.

### Modelo Cinemático
El movimiento se rige por las siguientes ecuaciones de velocidad lineal ($v$) y velocidad angular ($\omega$):

$$v = \frac{v_{r} + v_{l}}{2}$$
$$\omega = \frac{v_{r} - v_{l}}{L}$$

Donde:
* **$v_{r}$**: Velocidad de la rueda derecha 
* **$v_{l}$**: Velocidad de la rueda izquierda
* **L**: Distancia entre las ruedas

## Instrucciones de Ejecución
Para reproducir la simulación del robot móvil diferencial en Webots, siga los siguientes pasos:

1. Descargar e instalar el simulador :contentReference[oaicite:0]{index=0} desde su sitio oficial:  
   [Descargar Webots](https://www.cyberbotics.com/#download)  
   Luego, abrir la aplicación.  
  
2. Descargar e instalar **Python** desde el sitio oficial:  
   [Descargar Python](https://www.python.org/downloads/)  
   - Se utilizó la versión **Python 3.12.4** para el desarrollo de este laboratorio. 
   
   Asegurarse de que Python esté correctamente instalado y agregado al sistema.

3. Descargar o clonar este repositorio en su computador. Alternativamente, puede descargar únicamente el archivo del controlador (`lab1controlere-puck.py`) necesario para la simulación.

4. En Webots, abrir un mundo que contenga el robot móvil diferencial tipo *e-puck* (puede utilizar un ejemplo incluido en Webots).

5. Seleccionar el robot dentro del entorno de simulación.

6. En las propiedades del robot:
   - Ubicar el campo **controller**
   - Asignar el archivo `lab1controlere-puck.py` incluido en este repositorio  
   (para esto, asegúrese de que el archivo esté dentro de la carpeta del proyecto o en la ruta accesible por Webots).

7. Presionar el botón **Play** en Webots para iniciar la simulación.

## Resultados y Experimentos
Se realizaron diversas pruebas modificando las velocidades de los motores para observar la trayectoria resultante:

| Configuración | Tipo de Movimiento |
| :--- | :--- |
| $v_{r} = v_{l}$ | Movimiento rectilíneo  |
| $v_{r} \neq v_{l}$ | Trayectoria curva  |
| $v_{r} = -v_{l}$ | Rotación sobre su propio eje  |

### Desafíos Implementados
El controlador ha sido programado para ejecutar las siguientes figuras:
* Línea recta
```python
left_motor.setVelocity(2.0)
right_motor.setVelocity(2.0)
```
* Curva y círculo concéntrico
```python
left_motor.setVelocity(2.5)
right_motor.setVelocity(5.0)

if current_time >= start_time + tiempo_circulo:

            rectas = 0
            estado = 0
            start_time = current_time
```   

* Cuadrado
```python
if estado == 0:  # ESTADO: Recta

            left_motor.setVelocity(2.0)
            right_motor.setVelocity(2.0)

            if current_time >= start_time + tiempo_recta:

                estado = 1
                cont = cont + 1
                start_time = current_time

        elif estado == 1:  # ESTADO: Esquina

            left_motor.setVelocity(3.0)
            right_motor.setVelocity(0)

            if current_time >= start_time + tiempo_giro:
                estado = 0
                start_time = current_time
```  

* Tiempos
```python
tiempo_circulo = 8.0
tiempo_recta = 2.0
tiempo_giro = 1.45
tiempo_pausa = 5.0
```  

## Evidencia Visual
A continuación se muestra la ejecución del controlador del robot en Webots.
### Demostración del robot
![Robot en funcionamiento](Robot-Video.gif)

### Descripción del comportamiento observado

- Movimiento recto al aplicar velocidades iguales en ambas ruedas.
- Rotación sobre su propio eje al aplicar velocidades opuestas.
- Trayectorias curvas y circulares al utilizar velocidades distintas.
- Formación de figuras geométricas mediante cambios de estado en el controlador.

## Análisis de Resultados

#### 1. **¿Qué ocurre cuando ambas ruedas tienen la misma velocidad?**
Cuando $v_r = v_l$, el robot se desplaza en línea recta.  
Esto ocurre porque no existe diferencia de velocidades entre las ruedas, por lo que la velocidad angular ($\omega$) es igual a cero. En consecuencia, el robot no rota y mantiene una trayectoria rectilínea constante.

#### 2. **¿Cómo cambia la trayectoria cuando las velocidades son diferentes?**
Cuando $v_r \neq v_l$, el robot describe una trayectoria curva.  
La diferencia entre las velocidades genera una velocidad angular distinta de cero, provocando que el robot gire mientras avanza.  

El radio de la curva depende de la diferencia entre $v_r$ y $v_l$:  
- Si la diferencia es pequeña → curva suave  
- Si la diferencia es grande → giro más cerrado  

#### 3. **¿Qué ocurre cuando una rueda gira en sentido opuesto a la otra?**
Cuando $v_r = -v_l$, el robot realiza una rotación sobre su propio eje.  
En este caso, la velocidad lineal ($v$) es cero, mientras que la velocidad angular ($\omega$) es distinta de cero.  
Esto provoca que el robot gire en el mismo lugar, sin desplazamiento.

#### 4. **¿Qué tipo de movimiento permite dibujar un círculo?**
Para generar un movimiento circular, se deben mantener velocidades constantes y diferentes en ambas ruedas.  
Esto implica que tanto la velocidad lineal ($v$) como la velocidad angular ($\omega$) son constantes, generando una trayectoria circular uniforme.  

En la simulación, este comportamiento se logra con:
```python
left_motor.setVelocity(2.5)
right_motor.setVelocity(5.0)
```

### Resumen de Resultados
1.  **Velocidades iguales:** El robot mantiene un avance lineal ya que no existe diferencia de velocidades entre los actuadores que genere rotación.
2.  **Velocidades diferentes:** Se genera un radio de giro dependiente de la diferencia entre $v_{r}$ y $v_{l}$. Si una rueda es más veloz, el robot curva hacia el lado opuesto.
3.  **Velocidades opuestas:** Al girar en sentidos contrarios con la misma magnitud, el centro de masa del robot permanece estático mientras el chasis rota sobre su eje central.
4.  **Simulación de círculo:** Para lograr una trayectoria circular constante, se deben mantener velocidades diferentes pero fijas en ambas ruedas. Con el código proporcionado, el giro del robot es de 8 segundos.

## Conclusiones

En este laboratorio se logró comprender el comportamiento cinemático de un robot móvil diferencial mediante simulación en Webots.  
Se comprobó que el movimiento del robot depende directamente de las velocidades de sus ruedas, permitiendo generar trayectorias rectas, curvas, rotaciones y movimientos circulares.

Además, la implementación del controlador permitió validar el modelo teórico, mientras que la incorporación de perturbaciones evidenció diferencias entre un comportamiento ideal y uno más cercano a condiciones reales.

En conjunto, los resultados obtenidos confirman la relación entre el modelo cinemático y el comportamiento observado del robot.


