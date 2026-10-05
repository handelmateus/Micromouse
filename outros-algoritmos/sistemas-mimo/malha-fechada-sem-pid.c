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
	int Qamostras = 300; //Quantidade de amostras
	float REFV = 75 , REFW = PI/2, REFW_GRAU = 10; //Referencias de velocidade
	int vR, vL; //Medições de velocidades


	//Vetores de armazenamento
	float yR[Qamostras], rR[Qamostras], uR[Qamostras], eR[Qamostras]; //motor direito
	float yL[Qamostras], rL[Qamostras], uL[Qamostras], eL[Qamostras]; //motor esquerdo
	float yV[Qamostras], rV[Qamostras], uV[Qamostras], eV[Qamostras]; //velocidade linear
	float yW[Qamostras], rW[Qamostras], uW[Qamostras], eW[Qamostras]; //velocidade angular
	int u[Qamostras], ref[Qamostras];
	float tempo[Qamostras];


	//variável para alternar os modos entre coleta e envio de dados
	int estado = 0;

	// setando os motores como zero antes de iniciaro loop
	setMotores(0,0);
	// Loop principal
	while (1)
	{
		// Rodando o sistema em MALHA FECHADA
		setMotores(0,0);
		if (getSW1() == HIGH)
		{

			if(estado == 0)
			{
				delay_ms(1000); //delay para`dar tempo posicionar o micromouse
				float t0 = millis();

				for(k = 0; k < 1; k++)
				{
					//vetores de referencias das referencias linear e angular
					rV[k] = REFV;
					rW[k] = REFW_GRAU*(PI/180); // converte a referencia de grau para radiano
					rR[k] = 0; //referencia motor direito
					rL[k] = 0; //referencia motor esquerdo

					//setando as saídas iniciais
					yV[k] = 0;
					yW[k] = 0;

					//configurando os erros iniciais
					eV[k] = rV[k] - yV[k]; //erro da velocidade linear
					eW[k] = rW[k] - yW[k]; //erro de velocidade angular


				}

				for(k = 1; k < Qamostras; k++)
				{
					tempo[k] = (millis() - t0);

					//vetores de referencias das referencias linear e angular
//					float uWrad = REFW_GRAU*(PI/180);
					rV[k] = REFV;
					rW[k] = REFW_GRAU*(PI/180); // converte a referencia de grau para radiano

					//medindo as velocidades
					velRL(&vL, &vR); // medindo junto

					//saída
					yV[k] = normOut(velLin(vL, vR)); // saída normalizada
//					yV[k] = velLin(vL, vR); // velocidade linear em mm/s
					yW[k] = velAng(vL, vR); // velocidade angular em rad/s
//					yW[k] = velAngGrau(vL, vR); // velocidade angular em grau/s


					// recebendo o ERRO (malha fechada) e convertendo para o valor adequado a setMotores
					rR[k] = ref2ref(eV[k-1], eW[k-1], 0); // 0 = direita
					rL[k] = ref2ref(eV[k-1], eW[k-1], 1); // 1 = esquerda


					//transformando a referencia normalizada em valores entre 10 e 95 de pwm
					setMotores(normIn(rL[k]), normIn(rR[k]));

					//erro
					eV[k] = rV[k] - yV[k]; //erro da velocidade linear
					eW[k] = rW[k] - yW[k]; //erro de velocidade angular
				}
				estado = !estado;
			}
			else if(estado == 1)
			{
				beep(100);
				for(cont = 0; cont < Qamostras; cont++)
				{
					// Tudo que é R virou V e tudo que é L virou W
					printf("%d,%d,%d,%d,%d,%d\n", cont, amplify(rV[cont]), amplify(rW[cont]), amplify(yV[cont]), amplify(yW[cont]), amplify(tempo[cont]));
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