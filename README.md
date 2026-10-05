# Micromouse

Material de estudo e desenvolvimento de um robô **Micromouse** (robô que resolve labirintos): manuais e tutoriais, arquivos de fabricação das placas, simulador e programas auxiliares de identificação e controle. O firmware principal está no projeto `umart_lite_plus_teste` (README próprio); esta pasta reúne a documentação, o hardware, a simulação e os algoritmos de apoio.

## Estrutura

```
Micromouse/
├── Manuais-e-instrucoes/
├── outros-algoritmos/
│   └── sistemas-mimo/
├── Project-Futura-files/
└── Simulacao/
```

## Manuais-e-instrucoes

Documentação de apoio, organizada em quatro subpastas:

- **DEMAIS MANUAIS** – datasheets dos componentes (microcontrolador STM32F405, giroscópio LY3200ALH, driver de motor TB6612FNG, emissor e receptor infravermelho SFH4545 e TEFT4300) e dois PDFs do minicurso de Micromouse (projeto eletrônico e projeto mecânico).
- **INSTRUÇÕES INICIAIS PARA SOLUCIONAR O MAZE** – PDFs do minicurso sobre modelagem e resolução do labirinto e sobre o software controlador.
- **INSTRUÇÕES PARA CONSTRUIR LABIRINTO** – guias e modelos (SolidWorks e PDF) para montar um labirinto de testes, com dois projetos: um com peças de 12 mm e outro baseado no concurso português (UTAD). Inclui um arquivo de labirinto (`maze.mz2`).
- **INSTRUÇÕES PARA PREPARAR O IDE** – passo a passo para instalar e configurar o Eclipse com o compilador ARM, criar um projeto, gravar a placa via bootloader USB (DFU) e usar a placa uMart Lite Plus. Há também um roteiro de testes para validar LEDs, botão, buzzer, motores, encoders, sensores e giroscópio.

## outros-algoritmos

Programas em C para a uMart Lite Plus usados em **identificação de sistemas e sintonia de controladores** dos motores. Cada arquivo é uma versão alternativa do `main.c` (todos definem `main()` e incluem `main.h`), pensada para substituir o `main.c` do projeto `umart_lite_plus_teste` quando for testada. Eles não fazem parte do build do firmware.

| Arquivo | O que faz |
|---|---|
| `degrau-prbs-embarcado.c` | Aplica degrau com perturbação aleatória (PRBS) nos motores e coleta a resposta para identificação |
| `coleta-velocidade-linear-angular.c` | Coleta as velocidades linear e angular do robô, sem controladores |
| `astrom-embarcado.c` | PID discreto por motor, com ganhos obtidos pelo método do relé (Åström) |
| `pid-zn2-emabrcado.c` | PID por Ziegler-Nichols a partir de `Kcr` e `Tcr` |
| `pid-filtro-derivativo-3-patamares.c` | PID com filtro derivativo, com referência em três patamares |
| `media-5-medidas.c` | PID com filtro derivativo, medindo a velocidade das rodas pela média de 5 coletas |

A subpasta **sistemas-mimo** trata o robô como sistema de múltiplas entradas e saídas (velocidade linear e angular), com variações da malha de controle:

| Arquivo | O que faz |
|---|---|
| `malha-aberta-sem-pid.c` | Ensaio em malha aberta, sem controlador |
| `malha-fechada-sem-pid.c` | Malha fechada de velocidades, sem PID |
| `ma-global-com-pid.c` | PID global das velocidades, com perturbação PRBS na referência angular |
| `mf-com-pid-global.c` | Malha fechada com PID global das velocidades linear e angular |
| `pid-global-3-patamares.c` | PID global com referências em três patamares |
| `teste-controladores-siso.c` | Teste dos controladores SISO (um por motor e por velocidade) |

Em todos, os dados coletados são enviados pela serial ao final do ensaio, para análise externa.

## Project-Futura-files

Arquivos de fabricação (Gerber, versão V1.1) das placas de circuito impresso:

- **Futura_Gerber** – placa principal do projeto Futura (camadas de cobre, máscara, serigrafia e furação, mais o `.zip` pronto para envio à fábrica).
- **encoder_gerber** – placa do encoder, com os mesmos tipos de arquivos.

## Simulacao

Ferramentas para testar algoritmos sem o robô físico:

- **mms-c-master** – modelo em C para escrever um algoritmo de resolução de labirinto para o simulador MMS (exemplo de seguidor de parede esquerda), com instruções de configuração.
- **simulador (maze-solver-2022)** – simulador MMS para Windows (executável `MMS.exe` e bibliotecas Qt necessárias para rodá-lo).
