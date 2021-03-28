/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* En esta seccion se incluyen los archivos de cabecera (.h) que quieras usar en
   tu proyecto. Los archivos .h vienen siendo como una especie de interfaz que te 
   permite utilizar funciones y variables disponibles en otros archivos de codigo
   fuente o en bibliotecas ya compiladas (donde no ves el codigo pero tienes la 
   funcionalidad). 
   
 
   Cuando se genera un proyecto utilizando el STM32CubeMx, en la generacion del 
   codigo se incluyen estas "secciones" especiales, delimitadas usando comentarios
   con el formato USER ... BEGIN ... y USER ... END ...
   
   La idea es que, lo que agregues aqui se conserve cuando vuelvas a generar el 
   proyecto del Cube, es algo asi como, "esto no me lo borres". Tambien en el 
   proyecto del Cube hay una opcion que debe estar seleccionada para que esto se
   cumpla, en especifico es la opcion 'Keep user code when re-generating', disponible
   en las opciones de generacion del proyecto [Project Manager -> Code Generator]
   
   //ESTA OPCION ESTA COMO DEFAULT,POR LO QUE NO TENGO QUE IR A SELECIONARLA DIRECTAMENTE 
   
   La intencion con este proyecto es que te lleves una idea general de la estructura
   generada en el Cube y puedas comenzar a familiarizarte con este tema de la programacion
   y de manera inicial el uso del FreeRTOS (aunque los conceptos base son identicos para 
   casi todos los sistemas operativos de tiempo real). 
   
   Inicialmente, guiate por los comentarios para mantener un orden en la inclusion de nuevos
   elementos al programa, o sea, donde diga 'pon aqui las variables' pues haz eso. 
   
   Por convencion, los programas en lenguage C "comienzan" su ejecucion en una funcion 
   llamada main() [esto es una mentirilla piadosa, mas adelante veremos que no es tan asi]. 
   
   En esa funcion generalmente se inicializan los recursos a utilizar en la ejecucion del programa
   y luego se pasa a la ejecucion de las funcionalidades que quieres que haga tu firmware. 
   
   Tipicamente, un programa desarrollado para una PC esta pensado para brindar sus funcionalidades
   y cuando el usuario no las necesita mas lo cierra y fin del cuento. Este enfoque es valdido para 
   una PC pues en esta se pueden hacer infinidad de cosas diferentes dado su poder de procesamiento.
   
   En contraste, un sistema embebido a base de microcontroladores, generalmente esta pensado para la 
   realizacion de una o varias tareas especificas que no cambian de modo frecuente. Por ejemplo, en
   el caso de un horno de microondas, su funcion en general es calentar la comida, quizas tenga mil
   modos de trabajo pero es solamente eso, calentar la comida.
   
   En estos casos, los programas no terminan como los desarrollados para PC sino que siguen una estructura
   tipica como la siguiente:
   
   void main()
   {
        int entrada_sensor;
        int salida_actuador;
        
        // codigo de inicializacion
        // ...
        
        configurar_perifericos(); 
        
        // codigo de ejecucion de las funcionalidades
        // implementado por lo general como un ciclo infinito
        // ya que una vez en ejecucion solamente y siempre, se 
        // hara lo mismo
        
        while (1)
        {
            // hacer lo suyo aqui.
            
            // por ejemplo, "mirar" que pasa en el mundo exterior
            entrada_sensor = leer_sensores();
            
            // procesar esa informacion y actuar en consecuencia
            salida_actuador = procesar_datos(entrada_sensor);
            
            // y tomar una accion en respuesta a esos eventos y posiblemente otros de estado
            mover_actuadores(salida_actuador);
        }
   }
   
   A lo anterior se agrega el uso de interrupciones para responder a eventos externos, etc.
   Estas interrupciones al final lo que hacen es desencadenar la ejecucion de funciones 
   especiales, llamadas manejadores de interrupcion que no son llamadas por el desarrollador
   directamente, sino que es el compilador quien "por detras del telon" hace lo suyo para 
   que el micro vaya al lugar indicado ante estos eventos de interrupcion. Por lo general esta
   "magia" se apoya en el uso de nombres reservados para interrupciones o mediante el uso de 
   palabras claves (no en este caso pero por ejemplo __interrupt, ISR, etc., eso depende del micro
   y el compilador). Dado que las interrupciones no reciben ni devuelven datos su prototipo es el
   siguiente:
   
   //VI QUE VARIAS FUNCIONES DE FREERTOS(O MACROS PARA SER MAS ESPECIFICOS SI NO ME EQUIVOCO)
   //QUE SE EJECUTEN DENTRO DE LAS IT DEBEN SER DIFERENCIADAS,NORMALMENTE A;ADIENDOLE EL SUFIJO ISR
   
   
   void manejador_interrupcion(void)
   {
        int fuente_interrupcion;
        char dato;
        
        // aqui dentro quizas haya que identificar en concreto que interrupcion fue
        // en caso de haber una misma interrupcion para varias fuentes como en un puerto
        // serie que pide interrupcion tanto para rx como para tx.
        
        fuente_interrupcion = leer_fuente_interrupcion(); 
        
        // tomar decisiones en funcion de lo que se desee hacer
        if (fuente_interrupcion == Interrupcion_PS_Rx)
        {
            // fue por una recepcion de datos del puerto serie. hacer algo con el dato rx
            dato = leer_ps();
            
            // 'limpiar' la fuente de interrupcion en caso necesario
            limpiar_int_ps_rx();
        }
        
        // ... evaluar otras posibles interrupciones, etc. 
   }
   
   Haciendo una comparacion con el mundo de Arduino, que quizas hayas visto previamente, seria algo
   como lo siguiente: 
   
   
   void main()
   {
        // En el entorno tipico de Arduino, cuando creas un sketch, debes desarrollar tu proyecto usando
        // dos funciones predefinidas, setup() y loop() y queda oculto el resto del programa. En esencia
        // lo que pasa por detras del telon es algo como esto: 
        
        setup();    // Aqui haces tus inicializaciones de bibliotecas, de los perifericos que quieras usar, etc.
        
        while (1)
        {
            // aqui se llama continuamente la funcion loop() que es donde debes poner a ejecutar las funcionalidades
            // de tus proyecto en Arduino.
            loop();
        }
   }
   
   Este codigo tu no lo ves de forma predeterminada sino algo como esto:
   
   void setup()
   {
        
   }
   
   void loop()
   {
   
   }
   
   --------------------------------------------------------------------------------------------------------------------
   
   Esta filosofia del setup()/loop() o inicializacion/ciclo infinito es valida solamente cuando el proyecto no es muy 
   complejo, pero tiene el problema de no ser escalable, ademas de ser muy dificil de mantener cuando intentar agregar
   otras funcionalidades, etc, donde llega el momento que se vuelve una locura que no funciona como se espera. Es en este
   entorno que entran a jugar los sistemas operativos, en concreto los llamados de tiempo real. Casi siempre que se dice
   de tiempo real se piensa en algo muy rapido, casi instantaneo. Yo en lo particular prefiero pensarlo como sistemas 
   operativos de tiempo preciso, o sea, que cumpla con los requisitos temporales que me interesa tener (ahora esto no es fundamental)
   
   La idea con los sistemas operativos es "dividir" el trabajo en diferentes "tareas", encargadas de funciones bien definidas
   y solo a eso. Este enfoque permite una mejor organizacion del firmware y hace mucho mas facil poder resolver problemas en
   partes especificas del sistema sin necesidad de tocar otras que ya funcionan bien. En esencia es como si tuvieras varios programas
   ejecutandose "simultaneamente" (mentirilla piadosa aqui tambien a menos que tengas un procesador con varios nucleos) y con 
   posibilidades de intercambio de informacion entre ellos. Una tarea no es mas que una funcion parecida al main() en el sentido
   que puede tener una inicializacion y debe* tener un ciclo infinito donde hace lo suyo. El * es porque existen variantes pero es lo mas
   comun. 
   
   void tarea1(void* pvParam)
   {
        // inicializacion
        // ....
        
        
        // ciclo infinito
        for ( ; ; )
        {
            // hacer lo suyo aqui
            // ...
        }
   }
   
   void tarea2(void* pvParam)
   {
        // inicializacion tarea 2
        // ...
        
        // ciclo infinito tarea 2
        while (1)   // esta forma y el for (;;) son equivalentes.
        {
            // hacer lo suyo 2 aqui
            // ... 
        }
   }
   
   Es posible mandarle datos a las tareas en el momento en que son creadas, eso se hace a traves del argumento
   pvParam. Un uso tipico de esto puede ser la ejecucion de una tarea que necesite saber determinados datos antes
   de hacer su trabajo. 
   
   En estos casos, la forma del programa cambia a una similar a esta:
   
   void main(void)
   {
        // hacer inicializaciones por ejemplo del hardware o recursos globales de la aplicacion
        
        // crear las tareas que sean necesarias (esto no solo puede ser aqui, tambien en otros lugares se pueden 
        // crear nuevas tareas)
        
        crear_tarea(tarea1, "nombre_tarea1", prioridad, ...);
        
        crear_tarea(tarea2, "nombre2", prioridad, ...);
        
        // crear otras cosas utilizadas para la sincronizacion y comunicacion entre tareas
        // por ejemplo colas, semaforos, etc. 
        // ...
        
        // una vez creado todo lo requerido inicialmente, arrancar el planificador del sistema operativo
        
        arrancar_planificador_os();
        
        // una vez arrancado el planificador del SO la ejecucion del main finaliza y solamente se ejecutan
        // las tareas definidas previamente y dos tareas adicionales (de forma predeterminada). Estas dos 
        // tareas adicionales son la tarea IDLE (desocupado) que se ejecuta siempre que el micro no tenga 
        // mas nada que hacer, seria el equivalente al while (1) {} que habia antes en el main y otra tarea 
        // que lleva el control de los temporizadores "software" (mas adelante ...)
        
        // teniendo en cuenta lo anterior, si la ejecucion llega a este punto (cosa que NO deberia pasar) 
        // indica que hubo un problema a la hora de arrancar el SO, generalmente por falta de memoria, etc.
        
        si_llego_aqui_hubo_problemas();     // esto nunca deberia ejecutarse pero puede servir como medio 
                                            // de diagnostico de problemas. 
   }

 */
 
/* Como nos interesa utilizar el FreeRTOS debemos incluir sus archivos de cabecera */ 
#include "FreeRTOS.h"       // Siempre
#include "task.h"           // Para poder usar tareas


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*
    En esta seccion es recomendable poner los 'typedef' especificos del programa (aunque lo sugerido realmente es hacerlo en 
    ficheros .h (de cabecera) para que sea mas facil su utilizacion en otros archivos de codigo fuente, etc). 
    
    En esencia el typedef es una forma de crear un nuevo nombre o alias para un tipo de datos ya existente (que puede ser otro typedef tambien).
    Por ejemplo, existen un grupo de tipos de datos basicos int (enteros), float y double (numeros de punto flotante, o sea, con coma), 
    asi como otros tipos compuestos como estructuras y uniones (struct / union). El typedef lo que hace es crear un nombre nuevo, que 
    puede ser usado en lugar de esos tipos de datos predefinidos. 
    
    typedef int NUMERO_ENTERO;      // Esto crea un nuevo nombre llamado NUMERO_ENTERO, que dondequiera que el compilador se lo encuentre
                                    // sabra que se trata de un numero entero (en este caso con signo, o sea, puede incluir numeros positivos,
                                    // y negativos. 
                                    
    typedef unsigned int NUM_SIN_SIGNO;     // De modo similar al anterior pero solo admite numeros >= 0 y menores que 2^32-1 = 4294967295 asumiento 
                                            // 32 bits.
                                            
    Asi, una vez definidos esos typedef puedes decir en tu codigo algo como esto:
    
    void main(void)
    {
        NUMERO_ENTERO   mi_variable_entera = 0;
        
        ....
    }
    
 */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*

    En este espacio se sugiere poner los #define propios de la aplicacion, de forma analoga a los typedefs
    es preferible ponerlos en .h y solamente poner aqui los especificos a usar en este fichero.
    
    
    Un #define lo que hace es basicamente sustitucion de texto, o sea, dondequiera que aparezca el texto
    inmediatamente despues del #define se sustituye por el texto a su derecha. Por ejemplo:
    
    #define PI  3.14159
    
    esto crea una constante simbolica llamada PI y su 'valor' sera 3.14159, asi cuando tengas ese texto (PI)
    el el codigo el compilador lo que hace es poner 3.14159
    
    float area_circulo = PI * radio * radio; 
    
    esta linea de codigo, antes de ser compilada se convierte en lo siguiente:
    
    float area_circulo = 3.14159 * radio * radio;
    
    Esto se parece un poco al typedef pero no es lo mismo, la diferencia esta en que usando el typedef 
    el compilador puede realizar verificaciones de compatibilidad entre los tipos de datos que se maneja
    en el codigo y puede determinar los requisitos para 'crear' variables, etc. Esto no funciona para el #define
    que es solamente sustitucion de texto. La utilidad mas explotada del #define es la definicion de constantes 
    simbolicas como el ejemplo de PI y auxiliar en los procesos de compilacion condicional (decidir que partes 
    compilar y cuales no en dependencia de esos defines)

 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/*
    Este tema de los macros viene siendo una forma un poco primitiva de definir funciones pero que 
    no es recomendable. En esencia hace uso del #define para definir supuestas funciones. Por ejemplo:
    
    #define CALC_CUADRADO(x)    ((x) * (x))
    
    Este macro permite calcular el cuadrado de dos numeros que pueden ser enteros o de punto flotante.
    Aqui la x seria el 'argumento' del macro. Asi cuando el programa se dice lo siguiente:
    
    
    int lado_cuadrado = 10;
    int area_cuadrado = CALC_CUADRADO(lado_cuadrado);
    
    en realidad lo que procesa el compilador es esto:
    
    int area_cuadrado = ((lado_cuadrado) * (lado_cuadrado));
    
    o sea, sustituye no solamente el nombre del macro, sino tambien sus argumentos, en este caso la 'x' la cambia por
    el nombre de la variable 'lado_cuadrado'. 
    
    Es necesario siempre utilizar los parentesis al usar las variables de los macros, sino fuera asi pueden haber problemas
    con el orden de ejecucion de las operaciones. Por ejemplo CALC_CUADRADO(3 + 4) sin los parentesis rodeando la x en la definicion
    del macro se convertiria en:
    
    (3 + 4 * 3 + 4) y no en ((3 + 4) * (3 + 4)) cuyos resultados son: (3+12+4) y (7 * 7) respectivamente con el primero mal. 
 */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*
  Aqui agregar las variables privadas del proyecto. Si se desea poder usar estas variables desde otros ficheros se deben escribir asi:
  
  int mi_variable_global; 
  
  Para poder usar esta variable desde otro fichero de codigo fuente se debe usar:
  
  extern int mi_variable_global;
  
  ------------------------------------------------------------------------------------------
  
  Si por el contrario (que es lo recomendado) no se desea, se le agrega el calificador static en la declaracion:
  
  static int mi_variable_solo_para_este_fichero;
        
 */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/*

    Aqui se colocan los prototipos de funciones definidas en este archivo de codigo fuente.
    Un prototipo no es mas que el encabezado de la funcion, que incluye informacion sobre el tipo
    de datos que devuelve o void si no devuelve nada, el nombre de la funcion propiamente dicha y
    los argumentos que recibe. Para el caso de los argumentos basta con decir el tipo de datos
    aunque para mejor legibilidad es apropiado poner tambien el nombre de esos argumentos.
    
    Por ejemplo, en las lineas de arriba dice 
    
    void SystemClock_Config(void);
    
    Eso le avisa al compilador que en algun lugar de este fichero, tipicamente mas abajo sino no tiene sentido
    usar prototipos, hay una funcion llamada SystemClock_Config, que no devuelve nada ni tampoco recibe ningun
    argumento, por eso el void en ambos casos.
    
    La necesidad de esto viene dada por el hecho de que el compilador procesa el fichero desde arriba hacia 
    abajo y de izquierda a derecha, asi, si necesitas llamar alguna funcion el compilador debera saber que 
    devuelve, como se llama y que necesita antes de poderla incluir en tu codigo, el solo se encarga de buscarla
    luego. Por ejemplo:
    
    
    void main(void)
    {
        int m = 10;
        int n;
        
        
        n = evaluar_funcion(m);
        
        ...
    }
    
    int evaluar_funcion(int arg)
    {
        return arg + 10;
    }
    
    Cuando el compilador intenta procesar este fichero dara error pues el no conoce ninguna funcion con el nombre
    'evaluar_funcion' que reciba y devuelva un numero entero. Para evitar esto basta con agregar el prototipo de la funcion
    antes de utilizarla, asi:
    
    int evaluar_funcion(int arg);       // esto va delante del main o antes de la funcion que la llame a ella.
    
 */
 
 /* Prototipo de la funcion que representara la tarea_principal */
static void tarea_principal(void* parametro_sin_uso);   // static porque solo se usara en este fichero

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

    /*

    Aqui puedes agregar tu codigo, aunque por temas de organizacion te sugiero que sea debajo

    */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

    /* 
        Finalmente llegamos a la funcion main() :)
    
        En esta parte puede ir codigo de inicializacion.
    
        Las funciones que vienen a continuacion garantizan que los distintos 
        subsistemas del microcontrolador esten listos antes de que se ejecute tu codigo
        
        Recuerda que esto es un micro, no una PC, hay que configurar el hardware.
    
        La funcion HAL_Init() se encarga de 'mover la palanquitas' que permiten el uso de
        funciones de mas alto nivel para manejar los perifericos, etc.
    
        HAL quiere decir Hardware Abstraction Layer
    
        Luego de eso configurar los distintos 'relojes' usados por los perifericos.
        Esto lo agrega solo el Cube a partir de tus selecciones en la interfaz grafica.
        
        Por ejemplo, si quieres que el micro vaya a toda maquina escoges la mayor frecuencia.
        Si por el contrario te interesa tener el menor consumo de potencia posible, entoces
        debes usar la frecuencia minima posible, etc. 
        
        Para realizar esta configuracion se llama a la funcion SystemClock_Config(), cuyo prototipo
        aparece mas arriba. 
        
        Luego se inicializan los pines a utilizar (para un LED y un boton) y el puerto serie USART2
        para conversar con la PC, etc. 
        
        A continuacion se crea un tarea, llamada tarea_principal que hara la honrosa mision de flashear el
        LED y finalmente se arranca el planificador de tareas del FreeRTOS.
    
    */
    
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

    /* En esta parte crear la tarea que queremos y arrancar el scheduler */
    xTaskCreate(tarea_principal,                // Funcion que ejecuta el trabajo de la tarea principal
                "nombre_tarea_principal",       // nombre de la tarea. esto es solo para depuracion
                configMINIMAL_STACK_SIZE,       // cantidad de espacio de memoria a asignar a esa tarea
                NULL,                           // argumento a pasar a la tarea (ninguno en este caso, por eso NULL)    
                1,                              // prioridad (por ahora dejemoslo asi)
                NULL);                          // esto no se usa por ahora. en esencia permite tener una variable para 
                                                // poder controlar la tarea o mandarle informacion una vez corriendo.
                                                
    /* Arrancar el planificador (scheduler) del FreeRTOS */
    vTaskStartScheduler();
    
    /* En condiciones normales, la ejecucion no llega a este punto */
    

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* Aqui mi super tarea flasheadora de LEDs */
static void tarea_principal(void* parametro_sin_uso)
{
    int led_encendido;
    
    /* Como se menciono previamente, aqui es posible realizar
       alguna inicializacion de variables, perifericos, etc.
    */
    led_encendido = 0;
    
    /* Ciclo infinito para la tarea hacer lo suyo, en este caso flashear el LED */
    for ( ; ; )
    {
        if (led_encendido == 0)
        {
            // Actualizar variable de estado
            led_encendido = 1;
            
            // Mover las palancas del hardware para encender el LED (creo q prende con 1)
            // En este caso se llama a la funcion que permite 'escribir' un pin determinado
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        }
        else
        {
            // Actualizar variable de estado
            led_encendido = 0;
            
            // Idem al caso anterior, pero apagar en lugar de encender
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        }
        
        /* Agregar una demorita para poder ver el led flasheando. Si no se hace asi
           esta dificil ver algo
        */
        vTaskDelay(pdMS_TO_TICKS(1000));        // Hacer una media de 1000 ms antes de seguir.
        
        /* el pdMS_TO_TICKS es un macro que saca la cuenta de cuantos 'ticks' hacen falta para 
        llegar a la cantidad de milisegundos que se quiere. Por defecto cada tick dura 1ms pero se 
        puede cambiar */
        
        /* Una vez aqui, volver a hacer lo mismo, o sea, ir de nuevo a flashear el LED.

            Este pequeño programa es puramente didactico, no esta para nada optimizado. De hecho,
            en lugar de todo ese codigo de la variable de estado, etc, se simplifica usando la funcion
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        
            Esto puedes probarlo.
        */
    }
}

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
