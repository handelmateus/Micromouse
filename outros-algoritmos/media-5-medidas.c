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

    resetEncoderEsquerda();
    resetEncoderDireita();

    srand(time(NULL));

    //Variáveis
    float Ts = PERIODO_AMOSTRAGEM_MS; //período de amostragem
    int k = 0, cont = 0; //variaveis contadoras
    int Qamostras = 450; //Quantidade de amostras
    int REF = 50; //Sinal de referencia para controle
    int tempoMorto = 4; //Numero de peridos de amostragem de tempo morto
    int velDir = 0, velEsq = 0; // Variaveis para medição de velocidade

    //Vetores de armazenamento
    int yR[Qamostras], uR[Qamostras], eR[Qamostras]; //motor direito
	int yL[Qamostras], uL[Qamostras], eL[Qamostras]; //motor esquerdo
	int u[Qamostras], ref[Qamostras];
    float tempo[Qamostras];

    //Variaveis de controles obtidas pelo metodo do rele
    //    float g0R, g1R, g2R, g0L, g1L, g2L;
    //    g0R = 0.5545; g1R = -0.5846; g2R = 0.1717; //ganhos do PID do motor direito
    //    g0L = 0.5427; g1L = -0.5092; g2L = 0.1412; //ganhos do PID do motor esquerdo

    //Variáveis usadas com o filtro derivativo
    float p0R, p1R, p2R, q1R;
    float p0L, p1L, p2L, q1L;
    p0R = 0.3515; p1R = -0.5209; p2R = 0.1879; q1R = 0.8889;
    p0L = 0.3475; p1L = -0.5188; p2L = 0.1892; q1L = 0.8889;

    //variável para alternar os modos entre coleta e envio de dados
    int estado = 0;

    //setando os patamares
    for(k = 0; k < Qamostras/3; k++)
    {
    	ref[k] = 20;
    }
    for(k = Qamostras/3; k < 2*Qamostras/3; k++)
    {
    	ref[k] = 50;
    }
    for(k = 2*Qamostras/3; k < 3*Qamostras/3; k++)
    {
    	ref[k] = 30;
    }

    //Inicializando os vetores
    for(k = 0; k < tempoMorto; k++)
    {
    	// ref[k] = REF;
    	eR[k] = 0; eL[k] = 0;
    	uR[k] = 0; uL[k] = 0;
    	yR[k] = 0; yL[k] = 0;
    	tempo[k] = k*Ts;
    }

    setMotores(0,0);

    // Loop principal
    while (1)
    {
    	setMotores(0,0);
    	if (getSW1() == HIGH)
    	{
    		if(estado == 0)
    		{
    			beeps(2,50,50);
    			delay_ms(1000); //delay para`dar tempo posicionar o micromouse
    			float t0 = millis();
    			for(k = tempoMorto; k < Qamostras; k++)
    			{
    				// vetor de referencia
    				// ref[k] = REF;

    				// normalizando a saída
    				//    				velRL(&velEsq, &velDir);
    				velRL_media(&velEsq, &velDir);
    				yR[k] = normOut(velDir);
    				yL[k] = normOut(velEsq);

    				// calculando o erro
    				eR[k] = ref[k] - yR[k];
    				eL[k] = ref[k] - yL[k];

    				//referencia (normalizada) para a entrada a partir de Astrom
    				uR[k] = PID_FD(q1R, p0R, p1R, p2R, uR[k-1], uR[k-2], eR[k], eR[k-1], eR[k-2]);
    				uL[k] = PID_FD(q1L, p0L, p1L, p2L, uL[k-1], uL[k-2], eL[k], eL[k-1], eL[k-2]);
    				setMotores(normIn(uL[k]), normIn(uR[k])); //transformando a referencia normalizada em valores entre 10 e 95 de pwm

      				delay_ms(6);
//    				tempo[k] = (millis() - t0);
    				tempo[k] = k*Ts;
    			}
    			estado = !estado;
    		}
    		else if(estado == 1)
    		{
    			beep(100);
    			for(cont = 0; cont < Qamostras; cont++)
    			{
    				printf("%d,%d,%d,%d,%d,%d\n", cont, uR[cont], uL[cont], yR[cont], yL[cont], (int)(10*tempo[cont]));
    			}
    			estado = !estado;
    			beep(500);
    		}
//    		estado = !estado;
    	}

    } // fim do while(1)
}


/**
  * @brief Esta função é chamada a cada 1ms por interrupção
  */
void systick(void)
{
    // Função de interrupção do SysTick
}
