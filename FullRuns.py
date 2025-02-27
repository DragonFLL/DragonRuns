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



class PathPlanner:
    def __init__(self, largura, altura):
        self.largura = largura
        self.altura = altura
        self.missoes = []
        self.x_atual = 0# Posição inicial no eixo X -
        self.y_atual = 0# Posição inicial no eixo Y

    def adicionar_missao(self, missao):
        self.missoes.append(missao)

    def calcular_rota(self, coordenada_atual, missao_proxima):
        delta_x = missao_proxima.coordenadas[0] - coordenada_atual[0]
        delta_y = missao_proxima.coordenadas[1] - coordenada_atual[1]
        distancia = math.sqrt(delta_x**2 + delta_y**2)
        angulo = math.degrees(math.atan2(delta_y, delta_x))
        return round(distancia, 2), round(angulo, 2)

    def converter_distancia_para_graus(self, distancia_cm):
        return int((distancia_cm / 5) * 103)

    def calcular_distancia(self, x_atual, y_atual, x_destino, y_destino):
        return sqrt((x_destino - x_atual) ** 2 + (y_destino - y_atual) ** 2)

    def calcular_angulo(self, x_atual, y_atual, x_destino, y_destino):
        angulo_rad = atan2(y_destino - y_atual, x_destino - x_atual)
        return degrees(angulo_rad)

    async def mover_para_ponto(self, x_m, y_m, dec = 'NO', qnt = 920):
        # Atualiza a posição atual do robô
        x_inicial, y_inicial = self.x_atual, self.y_atual

        # Calcula a distância e o ângulo até o ponto de destino
        distancia_cm = self.calcular_distancia(x_inicial, y_inicial, x_m, y_m) # Cada unidade = 2 cm
        angulo_graus = int(self.calcular_angulo(x_inicial, y_inicial, x_m, y_m))

        if angulo_graus == 0:# Evitar divisão por zero
            motor_pair.move_for_degrees(motor_pair.PAIR_1, self.converter_distancia_para_graus(distancia_cm), 0, velocity=spike.velocidade)
        else:
            # Gira o robô para alinhar com o ângulo desejado
            motion_sensor.reset_yaw(0)
            print("Angulo em Graus {}".format(angulo_graus))

            await spike.Girar(angulo_graus)
            # time.sleep(0.2)
            
        if dec == 'YES':

            await motor_pair.move_for_degrees(motor_pair.PAIR_1, self.converter_distancia_para_graus(distancia_cm), 0, velocity=spike.velocidade, deceleration=qnt)

        elif dec == 'NO' or any:
            # Move o robô para frente pela distância calculada
            await motor_pair.move_for_degrees(motor_pair.PAIR_1, self.converter_distancia_para_graus(distancia_cm), 0, velocity=spike.velocidade)


        # Atualiza a posição atual após o movimento
        # Calculando a nova posição
        # Atualiza a posição X do robô
        # A nova posição X é calculada somando:
        # - A posição inicial X Inicial (onde o robô estava no começo)
        # - A distância que o robô andou na direção do ângulo, mas só na parte horizontal (X).
        # Para isso, usamos:
        # - math.cos(): Ajuda a descobrir quanta da distância está na direção horizontal.
        # - math.radians(): Transforma o ângulo (em graus) para um formato que o programa entenda.
        self.x_atual = x_inicial + distancia_cm * math.cos(math.radians(angulo_graus))# Atualiza posição X

        # Atualiza a posição Y do robô
        # A nova posição Y é calculada somando:
        # - A posição inicial Y (onde o robô estava no começo)
        # - A distância que o robô andou na direção do ângulo, mas só na parte vertical (Y).
        # Para isso, usamos:
        # - math.sin(): Ajuda a descobrir quanta da distância está na direção vertical.
        # - math.radians(): Transforma o ângulo (em graus) para um formato que o programa entenda.
        self.y_atual = y_inicial + distancia_cm * math.sin(math.radians(angulo_graus))# Atualiza posição Y
        print("Posição atual: X = {}, Y = {}".format(self.x_atual, self.y_atual))

    async def resetar_pos(self):
        self.x_atual = 0

        self.y_atual = 0

    async def imprimir_posicao(self):
        while True:
            print("Posição atual: X = {}, Y = {}".format(self.x_atual, self.y_atual))
            await runloop.sleep_ms(700)# Espera 0.7 segundos


# Classe principal do robô que controla movimentos e sensores
class Robot(PathPlanner):

    def __init__(self,control_pid, largura, altura):
        super().__init__(largura,altura)
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
    async def _movimento(self, pair, duration_s, acel, power = 800):
        # Definição dos parâmetros iniciais para aceleração e desaceleração:

        power_inicial = 100 # Potência inicial dos motores
        power_min = 100 # Potência mínima para desaceleração
        power_incremento = 100 # Incremento de potência para acelerar
        intervalo = 0.15 # Intervalo de tempo entre os ajustes de potência (em segundos)

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
        time.sleep(0.3)

    # Movimenta o robô para frente
    async def mover_forward(self, duration_s, acel, power = 500):
        await self._movimento(motor_pair.PAIR_1, duration_s, acel, power)

    # Movimenta o robô para trás
    async def mover_backward(self, duration_s, acel, power):
        await self._movimento(motor_pair.PAIR_1, duration_s, acel, -power)

    # Gira o robô em um ângulo específico (Direita ou esquerda)
    async def Girar(self, angulo, status = 'NO'):
        motion_sensor.reset_yaw(0)

        target_value = angulo

        p_constant = 0.6

        error = target_value - (motion_sensor.tilt_angles()[0] / 10)

        minimum_power = 200

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

        motor_pair.stop(motor_pair.PAIR_1, stop=HOLD)

        time.sleep(0.15)
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

            motor_pair.stop(motor_pair.PAIR_1, stop=BRAKE)

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


    # Função para o Robo parar de Mover
    async def Parar_de_Mover(self):
        motor_pair.stop(motor_pair.PAIR_1)

    # Função que retorna a cor atual lida pelo Sensor de Cor
    async def VerificarCorSensor(self, porta):
        cor = color_sensor.color(porta)
        return cor

    # Função que Retorna a intensidade da luz refletida pelo Sensor de Cor
    async def VerificarLuzRefletida(self, port):
        return color_sensor.reflection(port.C)

    # Função para Mostrar um grafico de Linhas com valor x e y alteraveis (Sem tela cheia)
    async def PlotarLineGraphic(self,valor_x, valor_y):
        linegraph.show(False)
        linegraph.plot(color.RED, valor_x, valor_y)

    # Função para limpar todos os Graficos de Linha
    async def LimparLineGraphic(self):
        linegraph.clear_all()

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

async def movimento(duration_seconds, power =400):
    motion_sensor.reset_yaw(0)

    time.sleep(0.3)

    start_time = time.ticks_ms() / 1000# Marca o tempo de início para o gráfico
    end_time = start_time + duration_seconds# Calcula o tempo final

    while time.ticks_ms() / 1000 < end_time:# Executa até o tempo fina5
        vel_direita, vel_esquerda, angulo_atual, AjusteFinal = PID(target=0, kp=0.7, ki=0.001, kd=0.7, power=power)

        motor_pair.move_tank(motor_pair.PAIR_1, vel_direita, vel_esquerda)

        # Ajustar o ângulo atual para o gráfico
        # angulo_plot = angulo_atual if angulo_atual != 0 else -1# Ajuste para o gráfico

        # Plotando o ângulo ajustado ao longo do tempo no gráfico
        elapsed_time = time.ticks_ms() / 1000 - start_time
        # linegraph.plot(color=color.RED, x=elapsed_time, y=angulo_atual)# Plotando o ângulo ajustado

        time.sleep(0.05)# Espera um pouco antes de repetir

    # Para os motores após o tempo especificado
    # motor_pair.move_tank(motor_pair.PAIR_1, 0, 0)
    motor_pair.stop(motor_pair.PAIR_1, stop=SMART_BRAKE)
    time.sleep(0.3)

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

            await spike.mover_para_ponto(23,0)

            await spike.mover_para_ponto(33.3, 4)

            await spike.mover_para_ponto(56.5, -0.5)


            await motor.run_for_time(port.B, 500, -400)

            await spike.Girar(34)
            # time.sleep(1)

            await motor.run_for_time(port.D, 800, 400)

            await motor_pair.move_for_time(motor_pair.PAIR_1, 780, 0, velocity=-600)

            await motor.run_for_time(port.D, 600, -400)

            await spike.Girar(-39)

            await motor.run_for_time(port.B, 500, 400)

            await motor_pair.move_for_time(motor_pair.PAIR_1, 670, 0, velocity=1000)

            time.sleep(0.2)

            await motor.run_for_time(port.B, 500, -400)

            await motor_pair.move_for_time(motor_pair.PAIR_1, 4700, 20, velocity=-1000)

            time.sleep_ms(4000)

            await movimento(1, 450)
            await spike.Girar(78)

            await motor.run_for_time(port.B, 500, 455)

            await movimento(1.35, 600)

            await spike.Girar(20)


            await movimento(3.6, 850)

            while True:
                    if button.pressed(button.RIGHT):

                        await spike.resetar_pos()

                        await spike.mover_para_ponto(25,0)


                        await spike.mover_para_ponto(46,-1.8)


                        await spike.mover_para_ponto(59,-4)

                        await motor.run_for_time(port.B,900, -500)

                        await motor_pair.move_for_time(motor_pair.PAIR_1, 2000, 0, velocity=-1000)
     async def L02(self):
            motor_pair.pair(motor_pair.PAIR_1, port.F, port.E)
            motor.reset_relative_position(port.D, 0)


            await movimento(0.53,900)
            await motor.run_for_time(port.B, 560, -300)
            await movimento(duration_seconds= 0.9, power=-900 )
            await motor.run_for_time(port.B, 490, 490 )
            time.sleep_ms(2500)

            await movimento(0.88, 900)
            await motor.run_for_time(port.B, 1350, -55)
            await motor.run_for_time(port.B, 700, 400)
            await movimento(0.4, -900)

            await spike.Girar(-90)


            await movimento(0.5, 800)
            await spike.Girar(90, 'YES')
            await movimento(0.77, 800)
            await spike.Girar(85)
            await motor.run_for_time(port.D, 300, 115)
            time.sleep_ms(200)


            await movimento(0.45, 650)
            await motor.run_for_time(port.D, 2500, -65)

            await movimento(1.7, -350)
            await movimento(0.5, 450)
            await spike.Girar(45)
    async def L03(self):
        motor_pair.pair(motor_pair.PAIR_1, port.F, port.E)
        motor.run_for_time(port.D, 1200, 450)
        await movimento(0.44,550)
        await spike.Girar(90,'YES')
        await movimento(3.4, 400)

        await motor.run_for_time(port.B, 600, 600)
        await movimento(0.45, 300)
        time.sleep_ms(800)
        # await movimento(0.1, 270)
        await movimento(0.18, 200)
        motor.run_for_time(port.D, 1850, -500)
        time.sleep_ms(650)

        await motor.run_for_time(port.B, 500, -400 )
        await movimento(0.30, 400)
        await movimento(1.3, -800)
        await spike.Girar(-25 )
        await movimento(2.5, -900)
    
    async def L04(self):

                motor_pair.pair(motor_pair.PAIR_1, port.F, port.E)

                await spike.Definir_Velocidade_Movimento(85)

                await spike.Definir_Velocidade_Anexos(35)

                await spike.mover_forward(1.35, 'YES', 1000)

                await spike.Girar(40)

                time.sleep(0.3)

                await motor_pair.move_for_time(motor_pair.PAIR_1, 1010, 0, velocity=1000, stop=BRAKE, acceleration=970)
                # await spike.mover_para_ponto(50, 20)

                await motor.run_for_time(port.B, 800, -250)

                await motor_pair.move_for_time(motor_pair.PAIR_1, 900, 0, velocity=-800)

                # await spike.mover_forward(0.6, -800)

                # await spike.Girar(5)

                await motor_pair.move_for_time(motor_pair.PAIR_1, 800, 0, velocity=1000, stop=BRAKE)

                await motor.run_for_time(port.B, 500, -250)

                await motor_pair.move_for_time(motor_pair.PAIR_1, 3000, -25, velocity=-1000)

    async def L05(self):
            await spike.Definir_Motores(parFrente, port.F, port.E)

            await spike.Definir_Velocidade_Movimento(60)

            await spike.Definir_Velocidade_Anexos(35)

            await spike.Manutencao_Guinada()

            #Entrega no navio
            await spike.mover_forward(0.6, 500)

            # Tempo para reposicionamento
            time.sleep(3)

            # Entrega do Navio
            await motor_pair.move_for_time(motor_pair.PAIR_1, 1400, 3, velocity=800)

            await motor.run_for_time(port.D, 700, -200)

            await motor_pair.move_for_time(motor_pair.PAIR_1, 2000, -2, velocity=400)
            await motor_pair.move_for_time(motor_pair.PAIR_1, 800, 0, velocity=-900)


            await motor.run_for_time(port.D, 400, 300)

            # await spike.mover_forward(0.4, -900)
            await motor_pair.move_for_time(motor_pair.PAIR_1, 350, 0, velocity=-900)

            await motor_pair.move_for_time(motor_pair.PAIR_1, 270, 100, velocity=600)

            time.sleep(0.3)
            await motor_pair.move_for_time(motor_pair.PAIR_1, 2000, -2, velocity=700)

            # await spike.mover_forward(1.99, 1000)

            time.sleep(0.5)

            # Retorno para Base Direita

            # await spike.mover_forward(0.73, -900)

            await motor_pair.move_for_time(motor_pair.PAIR_1, 990, 0, velocity=-1000)


            await spike.Girar(50)

            time.sleep(0.4)

            await motor_pair.move_for_time(motor_pair.PAIR_1, 1000, 0, velocity=900)

            await motor_pair.move_for_time(motor_pair.PAIR_1, 1700, 11, velocity=900)

            await spike.Girar(-30)

            await motor_pair.move_for_time(motor_pair.PAIR_1, 1380, 0, velocity=-1000, deceleration=666)

            await motor_pair.move_for_time(motor_pair.PAIR_1, 1400, 0, velocity=1000)



    async def L06(self):
            await spike.Definir_Motores(parFrente, port.F, port.E)

            await spike.Definir_Velocidade_Movimento(84)

            await spike.Definir_Velocidade_Anexos(35)

            await spike.Manutencao_Guinada()

            await spike.mover_para_ponto(58,0, 'YES', 200)

            await movimento(1.4, -800)


    async def L07(self):
        motor_pair.pair(motor_pair.PAIR_1, port.F, port.E)
        await motor_pair.move_for_time(motor_pair.PAIR_1, 2800, 0, velocity=600, stop=HOLD, acceleration=1000, deceleration=900)

        await spike.Girar(-30)
        
        await movimento(0.9)
        
        await motor.run_for_time(port.D, 700, 400)
        
        await movimento(1.5, -400)
        
        time.sleep_ms(700)
        
        await turn(-25, 250)
        
        motor_pair.move_for_time(motor_pair.PAIR_1, 3300, 20, velocity=-900)

    async def L08(self):
                motor_pair.pair(motor_pair.PAIR_1, port.F, port.E)

                motor.reset_relative_position(port.D, 0)
                await spike.Definir_Velocidade_Anexos(35)
                await movimento(1.8, 800 )

                await spike.Girar(80, 'no')
                
                await movimento(0.6, 800)
                
                await motor.run_for_time(port.D, 2500, 200)
                
                motor.run_for_time(port.D, 800, -200)

                # await movimento(0.7, -600)

                # await spike.Girar(30, 'YES')

                # await movimento(0.5, 750)
                # await spike.Girar(-35, 'YES')

                await movimento(0.55, 800)

                await spike.Girar(-45, 'NO')

                await movimento(0.78, 600)

                await spike.MoverMotor('descer', port.B, 1850)

                await movimento(0.55, -900)

                await spike.Girar(75, 'NO')

                await movimento(1.4, 700)
                
                await movimento(0.73, -800)




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
            time.sleep_ms(750)
            
            await lancamento.L01()

            motor_pair.unpair(motor_pair.PAIR_1)
            continue
        

        elif cor is color.RED:
            light.color(light.POWER, color.RED)
            print("Cor Vermelho - Lançamento 02")
            time.sleep_ms(900)
            await lancamento.L02()
            

            motor_pair.unpair(motor_pair.PAIR_1)
            continue

        elif cor is color.GREEN:
            light.color(light.POWER, color.GREEN)
            print("Cor Verde - Lançamento 03")
            time.sleep(0.7)
            await lancamento.L03()
            motor_pair.unpair(motor_pair.PAIR_1)
            continue

        elif cor is color.BLUE:
            light.color(light.POWER, color.BLUE)
            print("Cor Azul - Lançamento 04 e 05")
            status = 'NO'
            while status == 'NO':
                if button.pressed(button.RIGHT):
                    time.sleep(0.7)
                    await lancamento.L04()
                    motor_pair.unpair(motor_pair.PAIR_1)
                    status = 'YES'
                if button.pressed(button.RIGHT):
                    time.sleep(0.7)
                    await lancamento.L05()
                    motor_pair.unpair(motor_pair.PAIR_1)
                    status = 'YES'
            continue

        elif cor is color.MAGENTA:
            light.color(light.POWER, color.MAGENTA)
            print("Cor Magente - Lançamento 06 e 07")
            status = 'NO'
            while status == 'NO':
                if button.pressed(button.RIGHT):
                    time.sleep(0.7)
                    await lancamento.L06()
                    motor_pair.unpair(motor_pair.PAIR_1)
                    status = 'YES'
                if button.pressed(button.LEFT):
                    time.sleep(0.7)
                    await lancamento.L07()
                    motor_pair.unpair(motor_pair.PAIR_1)
                    status = 'YES'
            continue

        elif cor is color.YELLOW:
            light.color(light.POWER, color.YELLOW)
            print("Cor Amarela - Lançamento 08")
            time.sleep(0.7)
            await lancamento.L08()
            motor_pair.unpair(motor_pair.PAIR_1)
            continue

runloop.run(main(), ButtonCollor())
