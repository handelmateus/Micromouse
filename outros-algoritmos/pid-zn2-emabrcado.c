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
    int Qamostras = 150; //Quantidade de amostras
    int REF = 30; //Sinal de referencia para controle
    //Vetores de armazenamento
    int yR[Qamostras], uR[Qamostras], eR[Qamostras]; //motor direito
	int yL[Qamostras], uL[Qamostras], eL[Qamostras]; //motor esquerdo
	int u[Qamostras], ref[Qamostras];
    float tempo[Qamostras];

    //Variaveis de controles obtidas pelo metodo do rele
    float KcrR, TcrR, KcrL, TcrL;
    KcrR = 1.4396; TcrR = 0.08;
    KcrL = 4.5506; TcrL = 0.08;

    //variável para alternar os modos entre coleta e envio de dados
    int estado = 0;

    //Inicializando os vetores
    for(k = 0; k<2; k++)
    {
    	ref[k] = REF;
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
    			delay_ms(1000); //delay para`dar tempo posicionar o micromouse
    			float t0 = millis();
    			for(k = 2; k < Qamostras; k++)
    			{
    				// vetor de referencia
    				ref[k] = REF;

    				// normalizando a saída
    				yR[k] = normOut(velRight());
    				yL[k] = normOut(velLeft());

    				// calculando o erro
    				eR[k] = ref[k] - yR[k];
    				eL[k] = ref[k] - yL[k];
    				//referencia (normalizada) para a entrada a partir de ZN2
    				uR[k] = pidZN2(Ts, KcrR, TcrR, uR[k-1], eR[k], eR[k-1], eR[k-2]);
    				uL[k] = pidZN2(Ts, KcrL, TcrL, uL[k-1], eL[k], eL[k-1], eL[k-2]);
    				setMotores(normIn(uL[k]), normIn(uR[k])); //transformando a referencia normalizada em valores entre 10 e 95 de pwm

      				delay_ms(6);
//    				tempo[k] = (millis() - t0);
    				tempo[k] = k*Ts;
    			}
    			estado = !estado;
    		}
    		else if(estado == 1)
    		{
    			for(cont = 0; cont < Qamostras; cont++)
    			{
    				printf("%d,%d,%d,%d,%d,%d\n", cont, uR[cont], uL[cont], yR[cont], yL[cont], (int)(10*tempo[cont]));
    			}
    			estado = !estado;
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



