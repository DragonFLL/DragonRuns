from hub import light_matrix, motion_sensor, port, button, light
import motor
import runloop, motor_pair, time, color_sensor, color
from app import linegraph
from motor import *
import math
from math import atan2, degrees, sqrt

# Definição de constantes que representam comandos de movimento
subir = 'subir'
descer = 'descer'
parFrente = motor_pair.PAIR_1 # Par de motores frontais

integral_sum = 0

last_error = 0

class Controle_PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp # Proporcional
        self.ki = ki # Integral
        self.kd = kd # Derivativo
        self.integral_sum = 0 # Soma do erro integral
        self.last_error = 0 # Erro anterior usado no cálculo do derivativo

    # Obtém o ângulo atual da guinada(yaw) do robô
    async def _get_current_yaw(self):
        return motion_sensor.tilt_angles()[0]

    # Calcula o erro atual com base no ângulo alvo
    async def _compute_error(self, target):
        current_angle = await self._get_current_yaw()
        return target - current_angle

    # Calcula o termo derivativo (taxa de mudança do erro)
    async def _compute_integral(self, error):
        self.integral_sum += error
        return self.integral_sum

    # Calcula o termo derivativo (taxa de mudança do erro)
    async def _compute_derivative(self, error):
        derivative = error - self.last_error
        self.last_error = error
        return derivative

    # Função principal que calcula a correção PID para os motores
    async def calculate(self, target, power):
        error = await self._compute_error(target)
        integral = await self._compute_integral(error)
        derivative = await self._compute_derivative(error)

        # Aplica a fórmula PID: (kp * erro) + (ki * integral) + (kd * derivativo)
        correction = (self.kp * error) + (self.ki * integral) + (self.kd * derivative)

        # Ajusta as velocidades dos motores com base na correção
        left_speed = power - correction
        right_speed = power + correction

        return int(right_speed), int(left_speed) # Retorna o ângulo atual

class Missao:
    def __init__(self, coordenadas, angulo):
        self.coordenadas = coordenadas# coordenadas como par ordenado [x, y]
        self.angulo = angulo

# Classe principal do robô que controla movimentos e sensores
class Robot():

    def __init__(self,control_pid, largura, altura):
        self.velocidade = 0
        self.velocidadeDosAnexos = 0
        self.controle_pid = control_pid
        self.largura = largura
        self.altura = altura

    # Define os motores conectados nas portas especificadas
    async def Definir_Motores(self, pair, porta1, porta2):
        motor_pair.pair(pair, porta1, porta2)

    # Define a velocidade de movimento do robô
    async def Definir_Velocidade_Movimento(self, velocidade = 50):
        self.velocidade = velocidade * 10

    # Define a velocidade dos anexos (motores adicionais)
    async def Definir_Velocidade_Anexos(self, velocidade = 30):
        self.velocidadeDosAnexos = velocidade * 10

    # Reseta o ângulo de guinada para 0
    async def Manutencao_Guinada(self):
        motion_sensor.reset_yaw(0)

    # Função genérica para mover o robô com aceleração e desaceleração suave
    async def _movimento(self, duration_s, acel, power = -800):
        # Definição dos parâmetros iniciais para aceleração e desaceleração:

        power_inicial = 100 # Potência inicial dos motores
        power_min = 100 # Potência mínima para desaceleração
        power_incremento = 100 # Incremento de potência para acelerar
        intervalo = 0.05 # Intervalo de tempo entre os ajustes de potência (em segundos)

        # Reseta o ângulo de guinada para garantir um movimento estável
        await self.Manutencao_Guinada()

        # Marca o tempo de início e calcula o tempo final da movimentação
        start_time = time.ticks_ms() / 1000 # Tempo atual em segundos
        end_time = start_time + duration_s # Tempo total de movimentação (start + duração)

        # Define o ponto em que a desaceleração deve começar (0.7s{700ms} antes do final)
        deceleration_start = end_time - 1

        # Inicializa a potência atual com a potência inicial definida
        current_power = power_inicial

        # Loop que executa o movimento até o tempo final definido
        while time.ticks_ms() / 1000 < end_time:
            # Calcula as velocidades corrigidas usando o controle PID
            left_speed, right_speed= await self.controle_pid.calculate(0, power)

            if acel == 'YES':
                motor_pair.move_tank(motor_pair.PAIR_1, left_speed, right_speed, acceleration=900)
            else:
                # Move o robô usando as velocidades calculadas
                motor_pair.move_tank(motor_pair.PAIR_1, left_speed, right_speed)

        
            time.sleep(intervalo)

        # Após o término do loop, para os motores
        motor_pair.stop(motor_pair.PAIR_1, stop=BRAKE)
        time.sleep(0.05)

    # Movimenta o robô para frente
    async def mover_forward(self, duration_s, acel, power = 500):
        await self._movimento(duration_s, acel, power)


    # Gira o robô em um ângulo específico (Direita ou esquerda)
    async def Girar(self, angulo, status = 'NO'):
        motion_sensor.reset_yaw(0)

        target_value = angulo

        p_constant = 0.3

        error = target_value - (motion_sensor.tilt_angles()[0] / 10)

        minimum_power = 170

        maximum_power = 500

        tolerance = 1

        while abs(error) > tolerance:

            while abs(error) > tolerance:
                error = target_value - (motion_sensor.tilt_angles()[0] / 10)

                
                slowdown_factor = max(1, abs(error) / target_value)
                # Aplica o fator ao controle de saída
                controller_output = (error * p_constant) * slowdown_factor

                # Garante que a potência está dentro dos limites
                controller_output = max(minimum_power, min(maximum_power, abs(controller_output))) * (-1 if error < 0 else 1)

                motor_pair.move_tank(motor_pair.PAIR_1, int(controller_output * -1), int(controller_output))

        motor_pair.stop(motor_pair.PAIR_1, stop=BRAKE)

        time.sleep(0.2)
        if status == 'YES':
            final_angle = motion_sensor.tilt_angles()[0] / 10
            error = target_value - final_angle

            while abs(error) > tolerance:

                while abs(error) > tolerance:
                    error = target_value - (motion_sensor.tilt_angles()[0] / 10)

                    # Calcula o fator de desaceleração baseado no erro
                    slowdown_factor = max(1, abs(error) / target_value)# Evita que fique menor que 20%

                    # Aplica o fator ao controle de saída
                    controller_output = (error * p_constant) * slowdown_factor

                    # Garante que a potência está dentro dos limites
                    controller_output = max(100, min(150, abs(controller_output))) * (-1 if error < 0 else 1)

                    motor_pair.move_tank(motor_pair.PAIR_1, int(controller_output * -1), int(controller_output))

            motor_pair.stop(motor_pair.PAIR_1, stop=HOLD)

            print("Ângulo final:", final_angle)
        print(motion_sensor.tilt_angles()[0])

    # Função para mover um motor específico em uma direção por um tempo determinado - Prioridade a Anexos
    async def MoverMotor(self, direcao: str, port1: int, duracao:int):

        # Define um multiplicador que ajusta o sentido do movimento do motor
        # Se a direção for 'subir', o multiplicador será 1 (movimento normal).
        # Se a direção for diferente de 'subir', o multiplicador será -1 (movimento inverso).
        multiplicador = lambda direcao: 1 if direcao == 'subir' else -1

        # Executa o motor na porta especificada (port1) por uma duração (duracao) e com uma velocidade ajustada.
        # A velocidade é definida pela variável `self.velocidadeDosAnexos` multiplicada pelo multiplicador,
        # que garante a direção correta.
        await motor.run_for_time(port1, duracao, self.velocidadeDosAnexos * multiplicador(direcao))
        motor.stop(port.A, stop=HOLD)


    # Função que retorna a cor atual lida pelo Sensor de Cor
    async def VerificarCorSensor(self, porta):
        cor = color_sensor.color(porta)
        return cor

    # Função que Retorna a intensidade da luz refletida pelo Sensor de Cor
    async def VerificarLuzRefletida(self, port):
        return color_sensor.reflection(port.C)

# Instancia o Controle PID com valores iniciais
pid = Controle_PID(0.65, 0.01, 0.4)

# Instancia do robô passando o controle PID e as medidas da arena para o PathPlanner - em cm.
spike = Robot(pid, 220, 140)

    # Função para o PID básico
def PID(target, kp, ki, kd, power):
    global integral_sum, last_error

    # Cálculo do erro (Erro = Target - Ângulo Atual)
    angulo_atual = motion_sensor.tilt_angles()[0]
    error = target - angulo_atual

    # Parte integral acumulada
    integral_sum += error

    # Parte derivativa (erro atual menos o erro anterior)
    derivative = error - last_error
    last_error = error

    # Correções Proporcional, Integral e Derivativa
    Correction = kp * error
    CorrectionIntegral = ki * integral_sum
    CorrectionD = kd * derivative

    # Somar todas as correções
    AjusteFinal = Correction + CorrectionIntegral + CorrectionD

    # Ajusta as velocidades baseadas no PID
    velocidade_direita = power - AjusteFinal
    velocidade_esquerda = power + AjusteFinal

    return int(velocidade_direita), int(velocidade_esquerda), angulo_atual, int(AjusteFinal) # Retorna o ângulo atual

async def movimento(duration_seconds, power=400, status="NO", kp=1, TARGET = 0):
    motion_sensor.reset_yaw(0)
    time.sleep(0.15)

    # power = max(950, power)

    start_time = time.ticks_ms() / 1000
    end_time = start_time + duration_seconds

    def perfil_aceleracao(t, total_time, max_power):
        meio = total_time / 3
        if t < meio:
            velocidade = ((t / meio) ** 3) * max_power# Aceleração gradual
        else:
            velocidade = max(0, max_power * (1.6 - ((t - meio) / (total_time - meio))))# Desaceleração gradual

        # Garante que a velocidade esteja no intervalo desejado
        return max(350, min(velocidade, 1000))

    while time.ticks_ms() / 1000 < end_time:
        elapsed_time = time.ticks_ms() / 1000 - start_time

        if status == "YES":
            current_power = perfil_aceleracao(elapsed_time, duration_seconds, power)
        else:
            current_power = power

        vel_direita, vel_esquerda, angulo_atual, AjusteFinal = PID(target=TARGET, kp=kp, ki=0.01, kd=1.5, power=current_power)

        motor_pair.move_tank(motor_pair.PAIR_1, vel_direita,  vel_esquerda)
        time.sleep(0.1)

    motor_pair.stop(motor_pair.PAIR_1, stop=SMART_BRAKE)
    time.sleep(0.1)


async def turn(degress, vel = 100):
    motion_sensor.reset_yaw(0)
    if (degress * -1) > 0:
        while round((motion_sensor.tilt_angles()[0] / 10)) != (degress * -1):
            print("TILT {}" .format(round(motion_sensor.tilt_angles()[0] / 10)))
            motor_pair.move_tank(motor_pair.PAIR_1, int((-vel * degress) / degress), int((vel * degress) / degress))
        print("oi")
        motor_pair.stop(motor_pair.PAIR_1)
    else:
        while round((motion_sensor.tilt_angles()[0] / 10)) != (degress * -1):
            print("TILT {}" .format(round(motion_sensor.tilt_angles()[0] / 10)))
            motor_pair.move_tank(motor_pair.PAIR_1, int((vel * degress) / degress), int((-vel * degress) / degress))
        print("oi")
        motor_pair.stop(motor_pair.PAIR_1)

    time.sleep(0.3)

class Lancamentos():

    async def L01(self):
            await spike.Definir_Motores(parFrente, port.F, port.E)

            await spike.Definir_Velocidade_Movimento(100)

            await spike.Definir_Velocidade_Anexos(35)

            await spike.Manutencao_Guinada()

            await movimento(0.6, 1000, 'YES')
            await spike.Girar(15)

            await movimento(0.4, 1000, 'YES')

            await spike.Girar(-10)


            await movimento(0.976, 700, 'YES')
            
            await spike.Girar(33)

            await motor.run_for_time(port.B, 500, -400)

            await motor.run_for_time(port.D, 800, 400)

            await movimento(0.46, -700)

            await motor.run_for_time(port.D, 600, -400)

            await spike.Girar(-38)
            
            await motor.run_for_time(port.B, 500, 400)

            await movimento(0.5, 1000, 'YES')

            await motor.run_for_time(port.B, 480, -400)

            await movimento(1.3, -1000)
            await motor_pair.move_for_time(motor_pair.PAIR_1, 1800, 3, velocity=-1000)


            time.sleep(2)

            await motor_pair.move_for_time(motor_pair.PAIR_1, 1800, 0, velocity=-1000)

            await motor_pair.move_for_time(motor_pair.PAIR_1, 2300, 0, velocity=1000)

            time.sleep(2.5)

            await movimento(0.65, 950)
            await spike.Girar(80)
            
            await movimento(0.85, 1000)            
            
            await spike.Girar(12)


            await motor_pair.move_for_time(motor_pair.PAIR_1, 4300, 0, velocity=1000)



            while True:
                    if button.pressed(button.RIGHT):
                        await movimento(0.6,1000)

                        await spike.Girar(-7.8)
                        await movimento(0.9, 600, 'YES', 1)

                        await spike.Girar(-5)

                        await movimento(0.5, 600)

                        await motor.run_for_time(port.B, 500, -300)

                        await motor_pair.move_for_time(motor_pair.PAIR_1, 5000, -2, velocity=-1000)

                        break
    async def L02(self):
            motor_pair.pair(motor_pair.PAIR_1, port.F, port.E)
            motor.reset_relative_position(port.D, 0)


            await movimento(0.52,900)
            await motor.run_for_time(port.B, 560, -300)
            await movimento(duration_seconds= 0.9, power=-900 )
            await motor.run_for_time(port.B, 490, 490 )
            time.sleep_ms(2500)

            await movimento(0.905, 800, 'NO')
            await motor.run_for_time(port.B, 1300, -45)
            await motor.run_for_time(port.B, 700, 400)
            await movimento(0.5, -900)

            await spike.Girar(-90)


            await movimento(0.55, 900)
            await spike.Girar(90)
            await movimento(1.07, 1000, 'YES')
            await spike.Girar(90, 'NO')
            await motor.run_for_time(port.D, 500, -350)
            time.sleep_ms(100)


            await movimento(0.56, 700)
            await motor.run_for_time(port.D, 1000, 120)


            await movimento(0.45, -900)

            await spike.Girar(-30)
            await movimento(0.95, 800, 'YES')

            await movimento(0.67, -900)

            await spike.Girar(-80)

            await movimento(2.4, -980)


            # await movimento(1.75, -900)

            

    async def L03(self):
        motor_pair.pair(motor_pair.PAIR_1, port.F, port.E)
        motor.run_for_time(port.D, 860, 390)
        await movimento(0.7, 700, 'NO')
        await spike.Girar(76)
        await movimento(1.655, 700, 'YES', 1)
        
        
        await motor.run_for_time(port.D, 1000, -590)


        # await motor.run_for_time(port.D, 750, 890)

        await movimento(0.5, 500)
 
        await movimento(1.7, -1000)
        # await spike.Girar(-25 
    
    async def L04(self):

                motor_pair.pair(motor_pair.PAIR_1, port.F, port.E)

                await spike.Definir_Velocidade_Movimento(85)

                await spike.Definir_Velocidade_Anexos(35)

                await movimento(1.2, 1000, 'YES')
                # await spike.mover_forward(1.35, 'YES', 1000)

                await spike.Girar(35)

                time.sleep(0.3)

                await movimento(0.6, 1000, 'YES')

                # await motor_pair.move_for_time(motor_pair.PAIR_1, 1010, 0, velocity=1000, stop=BRAKE, acceleration=970)
                # await spike.mover_para_ponto(50, 20)

                await motor.run_for_time(port.B, 800, -250)

                # await movimento(0.7, -1000)

                await motor_pair.move_for_time(motor_pair.PAIR_1, 900, 0, velocity=-1000)

                # await motor_pair.move_for_time(motor_pair.PAIR_1, 900, 0, velocity=-800)

                # await spike.mover_forward(0.6, -800)

                # await spike.Girar(5)

                await movimento(0.6, 1000, 'YES')

                # await motor_pair.move_for_time(motor_pair.PAIR_1, 800, 0, velocity=1000, stop=BRAKE)

                await motor.run_for_time(port.B, 400, -250)

                await motor_pair.move_for_time(motor_pair.PAIR_1, 700, 0, velocity=-1000)


                # await spike.Girar(-25)

                await motor_pair.move_for_time(motor_pair.PAIR_1, 2000, 0, velocity=-1000)


    async def L05(self):
            await spike.Definir_Motores(parFrente, port.F, port.E)

            await spike.Definir_Velocidade_Movimento(60)

            await spike.Definir_Velocidade_Anexos(35)

            await spike.Manutencao_Guinada()

            #Entrega no navio
            await spike.mover_forward(1.05, 'NO', 200)

            # Tempo para reposicionamento
            time.sleep(3)

            # Entrega do Navio
            # await motor_pair.move_for_time(motor_pair.PAIR_1, 650, -, velocity=1000)

            await motor_pair.move_for_time(motor_pair.PAIR_1, 1300, 2, velocity=1000)


            # await movimento(1, 900, 'NO', 2, -42)
            await motor.run_for_time(port.D, 460, -300)

            await motor_pair.move_for_time(motor_pair.PAIR_1, 1800, 1, velocity=700)
            await motor_pair.move_for_time(motor_pair.PAIR_1, 800, 0, velocity=-900)


            await motor.run_for_time(port.D, 400, 300)

            # await spike.mover_forward(0.4, -900)
            await motor_pair.move_for_time(motor_pair.PAIR_1, 350, 0, velocity=-900)

            await motor_pair.move_for_time(motor_pair.PAIR_1, 300, 100, velocity=600)

            # time.sleep(0.3)
            await motor_pair.move_for_time(motor_pair.PAIR_1, 3100, -2, velocity=700)


            await motor_pair.move_for_time(motor_pair.PAIR_1, 1000, -2, velocity=-1000)


            await spike.Girar(50)

            time.sleep(0.4)

            await motor_pair.move_for_time(motor_pair.PAIR_1, 1000, 0, velocity=900)

            await motor_pair.move_for_time(motor_pair.PAIR_1, 3300, 12, velocity=1000)





    async def L06(self):
            await spike.Definir_Motores(parFrente, port.F, port.E)

            await spike.Definir_Velocidade_Movimento(84)

            await spike.Definir_Velocidade_Anexos(35)

            await spike.Manutencao_Guinada()

            await movimento(1.58, 1000,'YES')

            await movimento(2, -1000)


    async def L07(self):
        motor_pair.pair(motor_pair.PAIR_1, port.F, port.E)
        # await motor_pair.move_for_time(motor_pair.PAIR_1, 2900, 0, velocity=600, stop=HOLD, acceleration=1000, deceleration=900)

        await movimento(2.44, 600, 'YES')
        await spike.Girar(-25)
    
        await movimento(0.72)
        
        await motor.run_for_time(port.D, 960, 100, stop=HOLD)
        time.sleep(0.97)
        
        await movimento(0.8, -1000)
        
        # time.sleep_ms(700)
        
        await spike.Girar(45)
        
        await movimento(1.5, -1000)

        # await motor_pair.move_for_time(motor_pair.PAIR_1, 3400, 70, velocity=-900)
    async def L08(self):
                motor_pair.pair(motor_pair.PAIR_1, port.F, port.E)

                motor.reset_relative_position(port.D, 0)
                await spike.Definir_Velocidade_Anexos(35)
                await movimento(1.9, 1000, 'YES' )

                await spike.Girar(80, 'no')
                
                await movimento(0.88, 1000)

                await spike.Girar(-30, 'NO')

                await movimento(0.78, 600)

                await movimento(0.55, -900)

                await movimento(0.58, 900)

                await movimento(0.56, -900)

                await spike.Girar(50)

                await movimento(0.95, 700)
                
                await movimento(0.67, -800)



lancamento = Lancamentos()

# Função principal
async def main():

            # await spike.Definir_Velocidade_Movimento(60)

            await spike.Definir_Velocidade_Anexos(35)

            await spike.Manutencao_Guinada()

            light.color(light.POWER, color.PURPLE)

    #Função Secundária para armazenar toda a Logica de Sensor de Cor (Metodo dos Aneis)
async def ButtonCollor():
    while True:
        cor = color_sensor.color(port.C)

        if cor is color.AZURE:

            light.color(light.POWER, color.AZURE)
            print("Cor Azul Claro - Lançamento 01")
            time.sleep_ms(250)
            status = 'NO'
            while status == 'NO':
                if button.pressed(button.RIGHT):
                    time.sleep(0.2)
                    await lancamento.L01()
                    status = 'YES'
                    break
            

        elif cor is color.RED:
            light.color(light.POWER, color.RED)
            print("Cor Vermelho - Lançamento 02")
            time.sleep_ms(200)
            status = 'NO'
            while status == 'NO':
                if button.pressed(button.RIGHT):
                    time.sleep(0.2)
                    await lancamento.L02()
                    status = 'YES'
                    break
                if button.pressed(button.LEFT):
                    await movimento(1.2, -100)
                    await movimento(0.7, 800)
                    status = 'YES'

                    break


        # elif cor is color.GREEN:

            # light.color(light.POWER, color.GREEN)
            # print("Cor Verde - Lançamento 03")
            # time.sleep(0.2)
            # await lancamento.L03()


        elif cor is color.BLUE:
            light.color(light.POWER, color.BLUE)
            print("Cor Azul - Lançamento 04 e 05")
            status = 'NO'
            while status == 'NO':
                if button.pressed(button.RIGHT):
                    time.sleep(0.2)
                    await lancamento.L04()
                    status = 'YES'
                elif button.pressed(button.LEFT):
                    time.sleep(0.2)
                    await lancamento.L05()
                    status = 'YES'


        elif cor is color.MAGENTA:
            light.color(light.POWER, color.MAGENTA)
            print("Cor Magenta - Lançamento 06 e 07")
            status = 'NO'
            while status == 'NO':
                if button.pressed(button.RIGHT):
                    time.sleep(0.2)
                    await lancamento.L06()
                    status = 'YES'
                elif button.pressed(button.LEFT):
                    time.sleep(0.2)
                    await lancamento.L07()
                    status = 'YES'
    

        elif cor is color.YELLOW:
            light.color(light.POWER, color.YELLOW)
            print("Cor Amarela - Lançamento 08")
            time.sleep(0.2)
            await lancamento.L08()

        motor_pair.unpair(motor_pair.PAIR_1)
        continue


runloop.run(main(), ButtonCollor())
