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
	int Qamostras = 500; //Quantidade de amostras
	float REFV = 20 , REFW = 0, REFW_GRAU = 90; //Referencias de velocidade
	int margem_prbs = 10*(REFV/100); // margem percentual de varicao do prbs em torno do ponto de operacao
	float margem_prbs_grau = 10*(REFW_GRAU/100); // margem percentual de variacao aplicada a referencia em grau (10% da referencia)
	float  rgrau = 0; // armazena temporariamente a referencia escolhida aleatoriamente
	int vR, vL; //Medições de velocidades
	int tempoMorto = 2; //Numero de periodos de amostragem de tempo morto
	int32_t lf = 0, l = 0, r = 0, rf = 0; // Sensores de parede
	int wall = 0; // Tipo de parede
	int r_iv = 0; // Sinal apos a leitura dos sensores de parede
	float Kp_iv = 1, u_iv = 0; // Ganho proporcional da leitura do infravermelho


	//Vetores de armazenamento
	float yR[Qamostras], rR[Qamostras], uR[Qamostras], eR[Qamostras]; //motor direito
	float yL[Qamostras], rL[Qamostras], uL[Qamostras], eL[Qamostras]; //motor esquerdo
	float yV[Qamostras], rV[Qamostras], uV[Qamostras], eV[Qamostras]; //velocidade linear
	float yW[Qamostras], rW[Qamostras], uW[Qamostras], eW[Qamostras]; //velocidade angular
	int u[Qamostras], ref[Qamostras];
	float tempo[Qamostras];

    //Variaveis de ganhos obtidas pelo metodo do Astrom | corrigir e atualizar os ganhos
    float g0R, g1R, g2R, g0L, g1L, g2L;
    g0R = 2.2859; g1R = -3.4750; g2R = 1.3341; //ganhos do PID do motor direito
    g0L = 3.4699; g1L = -5.4859; g2L = 2.1822; //ganhos do PID do motor esquerdo

    // Variaveis de controle das velocidades globais
    float Kp_V = 0.1; // velocidade linear
    float Kp_W = 0.1; // velocidade angular

	//variável para alternar os modos entre coleta e envio de dados
	int estado = 0;

	// setando os motores como zero antes de iniciaro loop
	setMotores(0,0);

	// Loop principal
	while (1)
	{
		// Rodando o sistema em MALHA ABERTA com PID nos motores DIREITO e ESQUERDO
		setMotores(0,0);
		if (getSW1() == HIGH)
		{

			if(estado == 0)
			{
				beep(300);
				delay_ms(1000); //delay para`dar tempo posicionar o micromouse
				float t0 = millis();

				for(k = 0; k < tempoMorto; k++)
				{
					//vetores de referencias das referencias linear e angular
					rV[k] = REFV; //prbs(REFV, margem_prbs); // referencia com um prbs
					rgrau = prbs(REFW_GRAU, margem_prbs_grau); // referencia em grau com um prbs
					rW[k] = rgrau*(PI/180); // converte a referencia de grau para radiano

					rR[k] = 0; //referencia motor direito
					rL[k] = 0; //referencia motor esquerdo

					//vetores de controle dos motores direito e esquerdo
					uR[k] = 0; //controle motor direito
					uL[k] = 0; //controle motor esquerdo

					//setando as saídas iniciais
					yV[k] = 0; //saida vel linear
					yW[k] = 0; //saida vel angular
					yR[k] = 0; //saida vel motor direito
					yL[k] = 0; //saida vel motor esquerdo

					//configurando os erros iniciais
					eV[k] = rV[k] - yV[k]; //erro da velocidade linear
					eW[k] = rW[k] - yW[k]; //erro de velocidade angular
					eR[k] = rR[k] - yR[k]; //erro de velocidade motor direito
					eL[k] = rL[k] - yL[k]; //erro de velocidade motor esquerdo

				}

				for(k = tempoMorto; k < Qamostras; k++)
				{
					tempo[k] = (millis() - t0);

					// ------------------------------------------------------------------------------------
					//medindo as velocidades
					velRL(&vL, &vR); // medindo junto

					//saídas
					yR[k] = vR; //saida motor direito
					yL[k] = vL; //saida motor esquerdo

					// Velocidade Linear
					rV[k] = REFV; //prbs(REFV, margem_prbs); // referencia com um prbs
					yV[k] = normOut(velLin(yL[k], yR[k])); // saída normalizada
//					eV[k] = rV[k] - yV[k]; //erro da velocidade linear
//					uV[k] = Kp_V*eV[k]; //saida do controlador proporcional de velocidade linear

					// Velocidade Angular
					rgrau = prbs(REFW_GRAU, margem_prbs_grau); // referencia em grau com um prbs
					rW[k] = rgrau*(PI/180); // (novo: referencia é a saída do controlador proporcional do infravermelho) | converte a referencia de grau para radiano
					yW[k] = velAng(yL[k], yR[k]); // velocidade angular em rad/s
//					eW[k] = rW[k] - yW[k]; //erro de velocidade angular
//					uW[k] = Kp_W*eW[k]; //saida do controlador proporcional de velocidade angular


					// ------------------------------------------------------------------------------------

					// Setando referencias fixas para ver o desempenho dos controladores individuais
					rR[k] = 50; //ref2ref_corrigida(rV[k], rW[k], 0); // 0 = direita
					rL[k] = 50; //ref2ref_corrigida(rV[k], rW[k], 1); // 1 = esquerda

					//erro dos motores individualmente (normaliza a saida para estarem nas mesmas unidades)
    				eR[k] = rR[k] - normOut(yR[k]); //erro de velocidade motor direito
    				eL[k] = rL[k] - normOut(yL[k]); //erro de velocidade motor esquerdo

					//calculos dos sinais de controle dos motores L e R a partir de Astrom
    				uR[k] = PID(g0R, g1R, g2R, uR[k-1], eR[k], eR[k-1], eR[k-2]);
    				uL[k] = PID(g0L, g1L, g2L, uL[k-1], eL[k], eL[k-1], eL[k-2]);

					//transformando a referencia normalizada em valores entre 10 e 95 de pwm
    				setMotores(normIn(uL[k]), normIn(uR[k])); // entrando no motor o sinal de controle local normalizado


				}
				estado = !estado;

			}
			else if(estado == 1)
			{
				beep(100);
				for(cont = 0; cont < Qamostras; cont++)
				{
					// Tudo que é R virou V e tudo que é L virou W
//					printf("%d,%d,%d,%d,%d,%d\n", cont, amplify(rV[cont]), amplify(rW[cont]), amplify(yV[cont]), amplify(yW[cont]), amplify(tempo[cont]));
					printf("%d,%d,%d,%d,%d,%d\n", cont, amplify(uR[cont]), amplify(uL[cont]), amplify(normOut(yR[cont])), amplify(normOut(yL[cont])), amplify(tempo[cont]));
//					printf("%d,%d,%d,%d,%d,%d\n", cont, amplify(uR[cont]), amplify(uL[cont]), amplify(yR[cont]), amplify(yL[cont]), amplify(tempo[cont]));

				}
				estado = !estado;
				//				beeps(3,100,100);
			}

			delay_ms(1000);
			beeps(2,100,100);
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




