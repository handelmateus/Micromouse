#include "main.h"

/**
 * @brief Programa Principal
 */
int main(void)
{
	// Configuração e inicialização dos periféricos
	buzzerConfig();
	ledsConfig();
	botaoConfig();
	motoresConfig();
	sensoresConfig();
	encodersConfig();
	usart1Config();
	usbConfig();

	calibrateGyro(); // calibração do giroscopio
	calibrateWallSensors(); //calibração dos sensores de parede
	resetEncoderEsquerda();
	resetEncoderDireita();

	srand(time(NULL));

	//Variáveis
	float Ts = PERIODO_AMOSTRAGEM_MS; //período de amostragem
	int k = 0, cont = 0; //variaveis contadoras
	int Qamostras = 500; //Quantidade de amostras
	float REF = 50; // Referencia global
	float refR = REF , refL = REF; //Referencias de velocidade esquerda e direita
	int velDir = 0, velEsq = 0; // Variaveis para armazenar as velocidades
	int margem_rand = 5; // Margem de variacao do randomize

	//Vetores de armazenamento
	float yR[Qamostras], rR[Qamostras], uR[Qamostras], eR[Qamostras]; //motor direito
	float yL[Qamostras], rL[Qamostras], uL[Qamostras], eL[Qamostras]; //motor esquerdo
	float u[Qamostras], ref[Qamostras];
	float tempo[Qamostras];

	//variável para alternar os modos entre coleta e envio de dados
	int estado = 0;
	setMotores(0,0);

	// Loop principal
	while (1)
	{
		setMotores(0,0);

		if (getSW1() == HIGH)
		{

			if(estado == 0)
			{
				delay_ms(1000); //delay para dar tempo posicionar o micromouse
				float t0 = micros();
				for(k = 0; k < Qamostras; k++)
				{
					tempo[k] = (micros() - t0);

					// medindo a velocidade
					velRL(&velEsq, &velDir);

					// normalizando a saída
					yR[k] = normOut(velDir);
					yL[k] = normOut(velEsq);

					//referencia variante (normalizada) para a entrada
					uR[k] = prbs(refR, margem_rand);
					uL[k] = prbs(refL, margem_rand);
					//					uR[k] = REF;
					//					uL[k] = REF	;

					//transformando a referencia normalizada em valores entre 10 e 95 de pwm
					setMotores(normIn(uL[k]), normIn(uR[k]));

				}
				estado = !estado;
			}
			else if(estado == 1)
			{
				beep(50);
				for(cont = 0; cont < Qamostras; cont++)
				{
					printf("%d,%d,%d,%d,%d,%d\n", cont, amplify(uR[cont]), amplify(uL[cont]), amplify(yR[cont]), amplify(yL[cont]), amplify(tempo[cont]));
				}
				beeps(3, 50, 50);
				estado = !estado;

			}
			delay_ms(1000);
			beeps(2, 150, 50);
		}

	}
}


/**
 * @brief Esta função é chamada a cada 1ms por interrupção
 */
void systick(void)
{
	// Função de interrupção do SysTick
}


