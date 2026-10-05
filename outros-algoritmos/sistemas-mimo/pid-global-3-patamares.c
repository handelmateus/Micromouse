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
	int Qamostras = 600; //Quantidade de amostras
	float REFV = 50, REFW = 0, REFW_GRAU = 0, REFW_PERCENT = 60; //Referencias de velocidade
	int margem_prbs = 10.0*(REFV/100.0); // margem percentual de varicao do prbs em torno do ponto de operacao
	float margem_prbs_grau = 10.0*(REFW_GRAU/100.0); // margem percentual de variacao aplicada a referencia em grau (10% da referencia)
	float  rgrau = 0; // armazena temporariamente a referencia escolhida aleatoriamente
	float vR, vL; //Medições de velocidades
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
	float g0R, g1R, g2R, g0L, g1L, g2L, ressintonia_motor = 0.8;
	g0R = 2.2859*ressintonia_motor; g1R = -3.4750*ressintonia_motor; g2R = 1.3341*ressintonia_motor; //ganhos do PID do motor direito
	g0L = 3.4699*ressintonia_motor; g1L = -5.4859*ressintonia_motor; g2L = 2.1822*ressintonia_motor; //ganhos do PID do motor esquerdo

    // Variaveis de controle das velocidades globais
    float g0V, g1V, g2V, g0W, g1W, g2W, ressintonia_linear= 1, ressintonia_angular = 1;
    g0V = 0.5839*ressintonia_linear; g1V = -0.5913*ressintonia_linear; g2V = 0.1700*ressintonia_linear; //ganhos do PID de velocidade linear
    g0W = 1.9409*ressintonia_angular; g1W = -2.4019*ressintonia_angular; g2W = 0.7810*ressintonia_angular; //ganhos do PID do de velocidade angular

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

				// Setando 3 patamares
				for(k = 0; k < (Qamostras/3); k++)
				{
					rV[k] = (3.0/3.0)*REFV; //referencia linear
					rW[k] = (3.0/3.0)*REFW_PERCENT; //referencia angular
				}
				for(k = (Qamostras/3); k < (2*Qamostras/3); k++)
				{
					rV[k] = (3.0/3.0)*REFV; //referencia linear
					rW[k] = (3.0/3.0)*REFW_PERCENT; //referencia angular
				}
				for(k = (2*Qamostras/3); k < (3*Qamostras/3); k++)
				{
					rV[k] = (3.0/3.0)*REFV; //referencia linear
					rW[k] = (3.0/3.0)*REFW_PERCENT; //referencia angular
				}

				for(k = 0; k < tempoMorto; k++)
				{
					//vetores de referencias das referencias linear e angular
//					rV[k] = REFV; //prbs(REFV, margem_prbs); // referencia com um prbs
//					rgrau = REFW_GRAU; //prbs(REFW_GRAU, margem_prbs_grau); // referencia em grau com um prbs
//					rW[k] = rgrau*(PI/180); // converte a referencia de grau para radiano

					rR[k] = 0; //referencia motor direito
					rL[k] = 0; //referencia motor esquerdo

					//vetores de controle dos motores direito e esquerdo
					uR[k] = 0; //controle motor direito
					uL[k] = 0; //controle motor esquerdo
					uV[k] = 0; //controle vel lin
					uW[k] = 0; //controle w ang

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
//					velRL_media(&vL, &vR);
					velRL(&vL, &vR); // medindo junto

					//saídas
					yR[k] = vR; //saida motor direito
					yL[k] = vL; //saida motor esquerdo

					// Velocidade Linear
//					rV[k] = REFV; //prbs(REFV, margem_prbs); // referencia com um prbs
					yV[k] = normOut(velLin(yL[k], yR[k])); // saída normalizada
					eV[k] = rV[k] - yV[k]; //erro da velocidade linear
					uV[k] = PID(g0V, g1V, g2V, uV[k-1], eV[k], eV[k-1], eV[k-2]); //saida do controlador proporcional de velocidade linear

					// Velocidade Angular
//					rgrau = REFW_GRAU; //prbs(REFW_GRAU, margem_prbs_grau); // referencia em grau com um prbs
//					rW[k] = rgrau*(PI/180); // (novo: referencia é a saída do controlador proporcional do infravermelho) | converte a referencia de grau para radiano
					yW[k] = normOut_Angular(velAng(yL[k], yR[k])); // velocidade angular em rad/s
					eW[k] = rW[k] - yW[k]; //erro de velocidade angular
					uW[k] = PID(g0W, g1W, g2W, uW[k-1], eW[k], eW[k-1], eW[k-2]); //saida do controlador proporcional de velocidade angular
					float uW_rad_s = percentToRadS(uW[k]); // saida do controlador convertida para radianos

					// ------------------------------------------------------------------------------------

					// calculando as referencias para os motores L e R | convertendo para o valor adequado a setMotores | usando a funcao corrigida
					// As entradas em malha FECHADA sao as saidas dos controladores de V e W
					rR[k] = ref2ref_corrigida(uV[k], uW_rad_s, 0); // 0 = direita
					rL[k] = ref2ref_corrigida(uV[k], uW_rad_s, 1); // 1 = esquerda

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
//					printf("%d,%d,%d,%d,%d,%d\n", cont, amplify(uV[cont]), amplify(uW[cont]), amplify(yR[cont]), amplify(yL[cont]), amplify(tempo[cont])); // mandando as velocidades isoladas para ver se da menos ruido
					printf("%d,%d,%d,%d,%d,%d\n", cont, amplify(uV[cont]), amplify(uW[cont]), amplify(yV[cont]), amplify(yW[cont]), amplify(tempo[cont]));
//					printf("%d,%d,%d,%d,%d,%d\n", cont, amplify(rV[cont]), amplify(rW[cont]), amplify(yV[cont]), amplify(yW[cont]), amplify(tempo[cont]));
//					printf("%d,%d,%d,%d,%d,%d\n", cont, amplify(uR[cont]), amplify(uL[cont]), amplify(normOut(yL[cont])), amplify(normOut(yL[cont])), amplify(tempo[cont]));
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




