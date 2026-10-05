#include "main.h"

/**
 * @brief Algorítmo de testes e coleta de velocidade (linear e angular). Sem uso de controladores.
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
	int Qamostras = 150; //Quantidade de amostras
	float REFV = 40 , REFW = PI/2, REFW_GRAU = 45; //Referencias de velocidade
	int vR, vL; //Medições de velocidades


	//Vetores de armazenamento
	float yR[Qamostras], uR[Qamostras], eR[Qamostras]; //motor direito
	float yL[Qamostras], uL[Qamostras], eL[Qamostras]; //motor esquerdo
	float yV[Qamostras], uV[Qamostras], eV[Qamostras]; //velocidade linear
	float yW[Qamostras], uW[Qamostras], eW[Qamostras]; //velocidade angular
	int u[Qamostras], ref[Qamostras];
	float tempo[Qamostras];


	//variável para alternar os modos entre coleta e envio de dados
	int estado = 0;

	// setando os motores como zero antes de iniciaro loop
	setMotores(0,0);
	// Loop principal
	while (1)
	{
		setMotores(0,0);
		if (getSW1() == HIGH)
		{

			if(estado == 0)
			{
				delay_ms(1000); //delay para`dar tempo posicionar o micromouse
				float t0 = millis();
				for(k = 0; k < Qamostras; k++)
				{
					tempo[k] = (millis() - t0);

					//vetores de entradas das referencias linear e angular
//					float uWrad = REFW_GRAU*(PI/180);
					uV[k] = REFV;
					uW[k] = REFW_GRAU*(PI/180); // converte a referencia de grau para radiano

					//medindo as velocidades
					velRL(&vL, &vR); // medindo junto
					//					vR = velRight();
					//					vL = velLeft();
					//
					//					//saída
					//					yV[k] = normOut(velLin(vL, vR));
					//					yW[k] = velAng(vL, vR); // Normalizada dentro da própria função

					//referencia (normalizada) para a entrada
//					uR[k] = uV[k] + uW[k]; // mudar essa forma de calculo depois
//					uL[k] = uV[k] - uW[k];
//					uR[k] = (2*normIn(uV[k]) + COMPRIMENTO_MOUSE_M*uW[k])/2;
//					uL[k] = (2*normIn(uV[k]) - COMPRIMENTO_MOUSE_M*uW[k])/2;
					// recebendo a referencia e convertendo para o valor adequado a setMotores
					uR[k] = ref2ref(uV[k], uW[k], 0); // 0 = direita
					uL[k] = ref2ref(uV[k], uW[k], 1); // 1 = esquerda

//					printf("uR = %d, uL = %d\n", uR[k], uL[k]);
					//transformando a referencia normalizada em valores entre 10 e 95 de pwm
					setMotores(normIn(uL[k]), normIn(uR[k]));
//					setMotores(1000, 1000); // retirando a normalização do cálculo

					//medindo as velocidades
					//velRL(&vL, &vR); // medindo junto
//					vR = velRight();
//					vL = velLeft();

					//saída
					yV[k] = normOut(velLin(vL, vR)); // saída normalizada
//					yV[k] = velLin(vL, vR); // velocidade linear em mm/s
					yW[k] = velAng(vL, vR); // velocidade angular em rad/s
//					yW[k] = velAngGrau(vL, vR); // velocidade angular em grau/s

				}
				estado = !estado;
			}
			else if(estado == 1)
			{
				beep(100);
				for(cont = 0; cont < Qamostras; cont++)
				{
					// Tudo que é R virou V e tudo que é L virou W
					printf("%d,%d,%d,%d,%d,%d\n", cont, amplify(uV[cont]), amplify(uW[cont]), amplify(yV[cont]), amplify(yW[cont]), amplify(tempo[cont]));
					// printf("%d,%d,%d,%d,%d,%d\n", cont, uR[cont], uL[cont], yR[cont], yL[cont], (int)(10*tempo[cont]));
				}
				estado = !estado;
				beeps(3,100,100);
			}
			delay_ms(1000);
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




