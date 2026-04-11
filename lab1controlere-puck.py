"""lab1controlere-puck controller."""

import random
from controller import Robot

# 1. Inicializar el robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 2. Configurar los motores
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

# 3. Establecer posición en infinito para usar control de velocidad
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# 4. Definir la velocidad inicial (en radianes por segundo)
left_motor.setVelocity(2.0)
right_motor.setVelocity(2.0)

tiempo_circulo = 8.0
tiempo_recta = 2.0
tiempo_giro = 1.45
tiempo_pausa = 5.0
# Usaremos una sola variable para el estado: 0 para recta, 1 para giro, 2 para circulo
# Usaremos un cont para los lados del cuadrado
estado = 0
rectas = 4
cont = 0
start_time = robot.getTime()


while robot.step(timestep) != -1:
    current_time = robot.getTime()
    eps_L = random.uniform(-0.1, 0.1)
    eps_R = random.uniform(-0.1, 0.1)

    
    if rectas < 4: # Estado: Rectas 
        
            left_motor.setVelocity(2.0 + eps_L)
            right_motor.setVelocity(2.0 + eps_R)
            
            if current_time >= start_time + tiempo_recta:
            
                estado = 1
                rectas = rectas + 1
                start_time = current_time
                 
            elif estado == 1:  # ESTADO: Esquina
        
                left_motor.setVelocity(3.0)
                right_motor.setVelocity(-3.0)
    
                if current_time >= start_time + tiempo_giro:
                    estado = 0
                    start_time = current_time
    
    
    if estado == 2: # Estao: Hacer un circulo
    
        left_motor.setVelocity(2.5)
        right_motor.setVelocity(5.0)
        
        if current_time >= start_time + tiempo_circulo:
        
            rectas = 0       
            estado = 0
            start_time = current_time
                     
    if  rectas == 4 and cont < 4: # Estado : Hacer un cuadrado
    
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
            
    elif cont == 4 or rectas == 4: #ESTADO: Detenido
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        
        if current_time >= start_time + tiempo_pausa:
            estado = 2
            rectas = 4
            cont = 0 
            start_time = current_time
