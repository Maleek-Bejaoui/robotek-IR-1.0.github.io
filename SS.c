#include "SCOOP_V1.h"


#define CARRE(x) ((x) * (x))
#define TIMEOUT_HOLD 100 	//en 100aine de ms et < 600
#define RAW_MAX_SIZE 255


#define adrs_RC 0x41
#define adrs_DC 0x40
#define ina_config 0x007F   //0b : 0000 0000 0111 1111 (bit du D6->D3  pour config Nbr des ech/messure)[1111->128 ech/mesure (12bits) -> 68.10ms] 
#define ina_calibration 0x953B // calibration rechauffeur 1 (GPIO1 ) pour current_LSB=10 uA et R_SHUNT=0.1 LSB   | la meme calibration pour la 2emme rechauffeur (GPIO2) avec Rshunt de 0.01 ca donne un LSB de 100 uA
#define periode_ticks 20 // 20 ticks pour 1kHz pour frequence du PWM de deux reachauffeurs avvec une interruption chaque 20kHz


volatile uint16_t count_20k=0; // pour gere le PWM du chauffeur
//volatile uint16_t tic_25ms_temp=0; //Acquisition de la température
//volatile uint16_t tic_25ms_current=0; // Acquisition de la courant
static bool diag_PWM_DC = true ;
static bool diag_PWM_RC = true ;

//doigt chauffant :
volatile uint16_t count_pwm_DC = 0;
volatile uint16_t periode_ticks_DC = 10;    
volatile uint16_t ticks_haut_DC = 0; // Durée à l’état haut dans un cycle 

volatile uint16_t test=0;  // Uniquement pour valider différentes fonctionnalités
volatile uint16_t test2=0;  // Uniquement pour valider différentes fonctionnalités

// PWM pour DIAG 
uint32_t start_time = 0;
uint32_t t_actuel = 0;
uint16_t pwm_active_time = 10; // Durée d’activation du PWM en seconde

static bool pwm_active = false;          // false = OFF, true = ON
static uint32_t last_change_time = 0;    // temps de dernière bascule

// pour la mesure périodique du temperature et du courant
static uint32_t last_measure_temp = 0;
static uint32_t last_measure_courant = 0; 


uint16_t raw_Kp;
uint16_t raw_Ki;
uint16_t max_integrale;
uint16_t consigne;
uint16_t temp;
uint16_t freq_pwm;
uint8_t pwm_value;
uint16_t ticks_haut;
uint16_t last_echantillonnage_mesure = 0;
uint16_t last_echantillonnage_raw2 = 0;
uint16_t max_AD590 = 0;
uint16_t min_AD590 = 0;
uint16_t max_PT100 = 0;
uint16_t min_PT100 = 0;
uint16_t Ecart_AD590 = 0;
uint16_t Ecart_PT100 = 0;
uint16_t min_Pression = 0;
uint16_t max_Pression = 0;




ScoopFSM fsm; // Global FSM instance



/*uint8_t To_RAW_8bit(uint8_t valeur_SK, uint8_t raw_id) { // Enregistre une valeur SK codée sur 8 bits dans le buffer RAWn.
    uint8_t* ptrRAWH = NULL;
    uint8_t tmp_size = 0;
    ptrRAWH = get_RAW_ptr(raw_id);
    tmp_size= get_DP_size(raw_id+1);
    if (tmp_size + 1 > RAW_MAX_SIZE) return 0; // taille dépassée
    ptrRAWH[tmp_size] = valeur_SK;
    update_raw_size(raw_id, tmp_size + 1);
    return tmp_size + 1;
    }*/

void optical_system_enable(uint8_t duty_cycle_pucent) {
    //DDRB |= (1 << PB7); //fixée matériellement
    INIT_GPIO7(OUT); 
    // Timer2 en Fast PWM, non-inversé, prescaler = 1
    TCCR2 = 0; 
    TCCR2 = (1 << WGM21) | (1 << WGM20)   
          | (1 << COM21)                 
          | (1 << CS20);                 

    OCR2 = (uint8_t)(duty_cycle_pucent*2.55); // Rapport cyclique (0–255)
}

void optical_system_disable() {
    TCCR2 = 0;              // Stop Timer2
    //PORTB &= ~(1 << PB7);   
    WRITE_GPIO7(0);  // signal PWM bas
}


bool check_periode(uint16_t period_sec, uint32_t *last_measure_ref) {
  
    uint32_t t_s = ((uint32_t)(read_param(TIME_CCSDS_MIN) * 60)) + ((uint32_t)(read_param(TIME_CCSDS_100MS) / 10));
    // Gestion de l’overflow
    if (t_s < *last_measure_ref) {
        *last_measure_ref = t_s;
        return false;
    }
    else if ((t_s - *last_measure_ref) >= period_sec) {
        *last_measure_ref = t_s;
        return true;
    }

    return false;
}

void handle_pwm_DC(uint16_t T_ON_sec, uint16_t T_OFF_sec , uint16_t dutty_cycle)
{
    uint32_t t_s = ((uint32_t)(read_param(TIME_CCSDS_MIN) * 60)) +
                   ((uint32_t)(read_param(TIME_CCSDS_100MS) / 10));

    if (pwm_active) {
        // PWM est actif : vérifier si la durée ON est écoulée
        if ((t_s - last_change_time) >= T_ON_sec) {
            desactivate_PWM_DC(); // Désactiver le PWM;
            pwm_active = false;
            last_change_time = t_s;
        }

    } else {
        // PWM est inactif : vérifier si la durée OFF est écoulée
        if ((t_s - last_change_time) >= T_OFF_sec) {
            PWM_DC_parametres(dutty_cycle); 
            activate_PWM_DC(); // Activer le PWM
            pwm_active = true;
            last_change_time = t_s;
        }
    }
}

void verif_anomalie(uint16_t captEnabled) {
    // Vérification des anomalies
   /* if (((captEnabled >> 7) & 1) == 1 && (read_hk(5) ==0xFFFF || read_hk(5)==0x0000)) {
        // Code d'erreur : PT100_1 anomalie
        write_param(Error_Code, (read_param(MODE) << 12) | 0x0001); 
       
    }
    if (((captEnabled >> 6) & 1) == 1 && (read_hk(6) ==0xFFFF || read_hk(6)==0x0000)) {
       // Code d'erreur : PT100_2 anomalie
        write_param(Error_Code, (read_param(MODE) << 12) | 0x0002);
    }
    if (((captEnabled >> 5) & 1) == 1 && (read_hk(7) ==0xFFFF || read_hk(7)==0x0000)) {
         // Code d'erreur : AD590_1 anomalie
        write_param(Error_Code, (read_param(MODE) << 12) |0x0003);
    }
    if (((captEnabled >> 4) & 1) == 1 && (read_hk(8) ==0xFFFF || read_hk(8)==0x0000)) {
         // Code d'erreur : AD590_2 anomalie
        write_param(Error_Code, (read_param(MODE) << 12) | 0x0004);
    }
    if (((captEnabled >> 3) & 1) == 1 && (read_hk(9) ==0xFFFF || read_hk(9)==0x0000)) {
         // Code d'erreur : Pression anomalie
        write_param(Error_Code, (read_param(MODE) << 12) | 0x0005);
    }
    if (((captEnabled >> 2) & 1) == 1 && (read_hk(10) ==0xFFFF || read_hk(10)==0x0000)) {
        // Code d'erreur : OPT anomalie
        write_param(Error_Code, (read_param(MODE) << 12) | 0x0006);
    }
    // Vérification de la différence entre PT100_1 et PT100_2
    if (((captEnabled >> 7) & 1) && ((captEnabled >> 6) & 1)) {
        if (CARRE(read_hk(5) - read_hk(6) ) >= read_param(diff_max_PT100) ) {
            // Code d'erreur : Différence entre PT100_1 et PT100_2 trop importante
            write_param(Error_Code, (read_param(MODE) << 12) | 0x0007);
        }
    }*/
   ;
}

void prendre_mesures(uint16_t captEnabled)
{
    ADC_SetCS(ADC_SS_2);
    if(((captEnabled >>7 ) & 1 )==1)write_HK(ADC_readavrgChannel(ADC_CH2,read_param(moy_count_PT100)),5);
    if(((captEnabled >>6 ) & 1 )==1)write_HK(ADC_readavrgChannel(ADC_CH3,read_param(moy_count_PT100)),6);
    if(((captEnabled >>5 ) & 1 )==1)write_HK(ADC_readavrgChannel(ADC_CH0,read_param(moy_count_AD590)),7);
    if(((captEnabled >>4 ) & 1 )==1)write_HK(ADC_readavrgChannel(ADC_CH1,read_param(moy_count_AD590)),8);
    if(((captEnabled >>3 ) & 1 )==1)write_HK(ADC_readavrgChannel(ADC_CH4,read_param(moy_count_Pression)),9);
    if(((captEnabled >>2 ) & 1 )==1)write_HK(ADC_readavrgChannel(ADC_CH5,read_param(moy_count_OPT)),10);
}

void i2c_ERROR(){
    write_param(ERROR_I2C, TWSR); 
    write_param(MODE_SUIVANT, (uint16_t) ERROR);
}

void ScoopFSM_init(ScoopFSM *fsm) {
    fsm->currentState = BOOT;
    fsm->nextState = BOOT;
    write_param(MODE, (uint16_t)fsm->currentState); // Initialize the mode to BOOT
    write_param(MODE_SUIVANT, 0xFFFF); // Initialize the next mode to an invalid state
    State_Entry_Handlers[fsm->currentState](fsm); // Call the entry handler of the initial state
}

void ScoopFSM_transition(ScoopFSM *fsm, uint16_t param_mode_suivant) {
    if (param_mode_suivant != 0xFFFF || fsm->nextState != fsm->currentState) { // Transition have been requested
        //First, Health Check
        if (param_mode_suivant == 99) {
            fsm->nextState = END; // Transition to END state if 99 is read
        }
        else if (param_mode_suivant >= STATE_COUNT) {
            fsm->nextState = ERROR; // Transition to the next state
        } else if (param_mode_suivant != 0xFFFF) {
            fsm->nextState = (ScoopState)param_mode_suivant; // Set the next state
        }
        //Then, handle request
        State_Exit_Handlers[fsm->currentState](fsm); // Call the exit handler of the current state
        fsm->currentState = fsm->nextState; // Update the current state
        State_Entry_Handlers[fsm->currentState](fsm); // Call the entry handler of the new state
        write_param(MODE, (uint16_t)fsm->currentState); // Update the mode parameter
        write_param(MODE_SUIVANT, 0xFFFF); // Reset the next mode parameter
    }
}

void ScoopFSM_loop(ScoopFSM *fsm) {
    // Call the routine handler of the current state
    State_Routine_Handlers[fsm->currentState](fsm);
    ScoopFSM_transition(fsm, read_param(MODE_SUIVANT));
}

// ---- Etat BOOT ----
static void ScoopFSM_Boot_Entry(ScoopFSM *fsm) {
    write_param(CTRL_uC, 0x0000); // on eteind alim secondaire
}

static void ScoopFSM_Boot_Routine(ScoopFSM *fsm) {
    _delay_ms(100);
}

static void ScoopFSM_Boot_Exit(ScoopFSM *fsm) {
    // Vérification pour passer en ACTION
    if(fsm->nextState == STABILISATION){
        raw_Kp = read_param(RAW_Kp);
        raw_Ki = read_param(RAW_Ki);
        max_integrale = read_param(MAX_INTEGRALE_PI);
        freq_pwm = read_param(FREQ);
        consigne = read_param(CONSIGNE);

        if (raw_Kp == 0 || raw_Ki == 0 || max_integrale == 0 || freq_pwm == 0 ) {
            fsm->nextState = ERROR;
            write_param(ERROR_uC, 2); // Code d'erreur : paramètres STABILISATION invalides        
        }
    }
    // Vérification pour passer en DIAG_Simple
    else if (fsm->nextState == DIAG_Simple){
        if(read_param(Rapport_Cyclique_RC)==0 ){  // Compléter l’identification des paramètres d’entrée en mode DIAG_Simple
            fsm->nextState = ERROR;
            write_param(ERROR_uC, 1);// Code d'erreur : paramètres DIAG_Simple invalides
        }
    }
    // Vérification pour passer en DIAG_Meca
    else if (fsm->nextState == DIAG_Meca){
        if(false){  // Compléter l’identification des paramètres d’entrée en mode DIAG_meca
            fsm->nextState = ERROR;
            write_param(ERROR_uC, 1);// Code d'erreur : paramètres DIAG_meca invalides
        }
    }
    _delay_ms(2000);
}

// ---- Etat DIAG_Simple ----
static void ScoopFSM_Diag_Simple_Entry(ScoopFSM *fsm) {

    write_param(PERIODE_SHOT_HK, 0xFFFF); // deactivation du shot HK (¨deep sleep¨)

    //activation des alimentations
    write_param(CTRL_uC, 0xe000);
    _delay_ms(10);


    // configuration du rapport cyclique du PWM du doigt chauffant [frequence fixe 1k]
    PWM_DC_parametres(read_param(Rapport_Cyclique_DC)); 
    activate_PWM_DC();
    start_time= read_param(TIME_CCSDS_MIN)*60 + read_param(TIME_CCSDS_100MS)/10;
    pwm_active_time=10; // les secondes d'activation du PWM ?
    ADC_SetCS(ADC_SS_2);
    ina_init_gen( adrs_DC ,  ina_config ,  ina_calibration ) ; // config du INA209 pour SCOOP adress 0x40 (DC)
    diag_PWM_DC = false ;

}

static void ScoopFSM_Diag_Simple_Routine(ScoopFSM *fsm) {

    // mesure courant , deactiviation du PWM du doigt chauffant et configuration pour l'activation du PWM de reauchauffeur
    t_actuel=read_param(TIME_CCSDS_MIN)*60 + read_param(TIME_CCSDS_100MS)/10;

    if ((t_actuel - start_time >= pwm_active_time ) && !diag_PWM_DC ) {
        write_param(Courant_DC,get_current());
        _delay_ms(70);
        desactivate_PWM_DC();
        diag_PWM_DC=true;
        //config pour rechauffeur 
        diag_PWM_RC=false;
        ina_init_gen(adrs_RC ,  ina_config ,  ina_calibration ) ; // config du INA209 pour SCOOP adress 0x41 (DC)
        start_time = read_param(TIME_CCSDS_MIN)*60 + read_param(TIME_CCSDS_100MS)/10;
        PWM_RC_parametres(read_param(Rapport_Cyclique_RC)); // config du PWM du rechauffeur
        activate_PWM_RC(); 
    }

    // Avtiviation du PWM pour du rechauffeur pendant pwm_active_time secondes
    if ((t_actuel - start_time >= pwm_active_time ) && !diag_PWM_RC ) {
    write_param(Courant_RC,get_current());
    _delay_ms(70);
    desactivate_PWM_RC();
    diag_PWM_RC=true;
    write_param(RESERVE_DP_3,t_actuel-start_time); //just for test
    _delay_ms(100);
        // Vérification de l'alimentation
        //if((read_param(CTRL_uC) & (MASK_CTRL_uC_3V3_S | MASK_CTRL_uC_5V_S | MASK_CTRL_uC_VBATT_S)) != 0xe000){
        /*if((MASK_CTRL_uC_3V3_S | MASK_CTRL_uC_5V_S | MASK_CTRL_uC_VBATT_S) == 0xe000){
            write_param(Error_Code, 0x100A); // ERROR ID1 :  probleme d'alimentation
            fsm->nextState = ERROR;
        }*/

        shotHK(); // shot HK une seule fois pendant le DIAG_Simple 

        verif_anomalie(read_param(CAPTEUR_ENABLE)); // Vérification des anomalies HK

        fsm->nextState = END; // Transition vers l'état STABILISATION après le DIAG_Simple
    }

}

static void ScoopFSM_Diag_Simple_Exit(ScoopFSM *fsm) {
    desactivate_PWM_DC();
    desactivate_PWM_RC();
}

// ---- Etat DIAG_Meca ----
static void ScoopFSM_Diag_Meca_Entry(ScoopFSM *fsm) {

    start_time= read_param(TIME_CCSDS_MIN)*60 + read_param(TIME_CCSDS_100MS)/10;
    //activation des alimentations
    write_param(CTRL_uC, 0xe000);
    _delay_ms(10);

}

static void ScoopFSM_Diag_Meca_Routine(ScoopFSM *fsm) {
    t_actuel=read_param(TIME_CCSDS_MIN)*60 + read_param(TIME_CCSDS_100MS)/10;
    if(start_time+t_actuel >= read_param(DUREE_MAX_DIAG)) {
        if((read_param(Activer_Chauffe) >> 0 ) & 1)
        {
            PWM_DC_parametres(read_param(Rapport_Cyclique_DC)); // config du PWM du doigt chauffant
            activate_PWM_DC();// activation du PWM du doigt chauffant
            ina_init_gen( adrs_DC ,  ina_config ,  ina_calibration ) ;
            _delay_ms(70);
            write_param(Courant_DC,get_current()); // mesure du courant du doigt chauffant
            
        }

        if((read_param(Activer_Chauffe) >> 1 ) & 1)
        {
            PWM_RC_parametres(read_param(Rapport_Cyclique_RC)); // config du PWM du rechauffeur
            activate_PWM_RC();// activation du PWM du rechauffeur
            ina_init_gen( adrs_RC ,  ina_config ,  ina_calibration ) ;
            _delay_ms(70);
            write_param(Courant_RC,get_current()); // mesure du courant du doigt chauffant
        }

         _delay_ms(100);
        // Vérification de l'alimentation
        //if((read_param(CTRL_uC) & (MASK_CTRL_uC_3V3_S | MASK_CTRL_uC_5V_S | MASK_CTRL_uC_VBATT_S)) != 0xe000){
        if((MASK_CTRL_uC_3V3_S | MASK_CTRL_uC_5V_S | MASK_CTRL_uC_VBATT_S) != 0xe000){
            fsm->nextState = ERROR;
            write_param(Error_Code, (read_param(MODE) << 12) | 0x000A); // ERROR :  probleme d'alimentation 
            return;
        }
        else{
            
            prendre_mesures(read_param(CAPTEUR_ENABLE)); // Prendre les mesures des capteurs Enabled
            verif_anomalie(read_param(CAPTEUR_ENABLE)); // Vérification des anomalies
        }

    }
    else 
    {
        write_param(Error_Code, 0x2001); // Code d'erreur : durée maximale mode DIAG_meca dépassée
        fsm->nextState = ERROR;
        return;
    }
}

static void ScoopFSM_Diag_Meca_Exit(ScoopFSM *fsm) {
       desactivate_PWM_DC();
       desactivate_PWM_RC();
}

// ---- Etat STABILISATION ----
// Fonction appelée à l’entrée de l’état "STABILISATION" de la FSM (Finite State Machine)
static void ScoopFSM_STABILISATION_Entry(ScoopFSM *fsm) {

    // Lecture des paramètres de régulation depuis la mémoire ou les réglages
    raw_Kp = read_param(RAW_Kp);                  // Lecture du gain proportionnel brut
    raw_Ki = read_param(RAW_Ki);                  // Lecture du gain intégral brut
    max_integrale = read_param(MAX_INTEGRALE_PI); // Fréquence de mise à jour de l'intégrale (anti-windup)

    // Configuration d’un registre de contrôle uC, probablement pour activer une sortie ou un mode
    write_param(CTRL_uC, 0xe000);                 // Ex. : active un bit de contrôle matériel

    // Initialisation du régulateur PI avec les paramètres lus
    PI_Init(raw_Kp, raw_Ki, max_integrale);
    if (get_PIInit() == false) {
        fsm->nextState = ERROR;
        return;
    }

    // Lecture de la fréquence de PWM souhaitée (en Hz)
    freq_pwm = read_param(FREQ);

    // Calcul de la période entre deux appels (en ticks) selon la fréquence de PWM
    // Exemple : à 100 Hz → 20000 / 100 = 200 ticks entre deux interruptions

    //init et config du INA209 pour SCOOP adress 0x41
     ina_init_gen(adrs_RC ,  ina_config ,  ina_calibration ) ;

    // Activation d'une interruption à 20 kHz (qui déclenchera probablement le traitement PI)
    activate_ISR_20kHz();
}

// ---- Routine exécutée périodiquement dans l’état STABILISATION de la FSM ----
static void ScoopFSM_STABILISATION_Routine(ScoopFSM *fsm) {
    // Vérifie que les alimentations 3.3V et 5V sont bien activées.
    // Si ce n’est pas le cas, passage à l’état d’erreur.
    if ((read_param(CTRL_uC) & (MASK_CTRL_uC_3V3_S | MASK_CTRL_uC_5V_S)) != 0xc000) {
        fsm->nextState = ERROR;        // Prochaine transition vers l'état ERROR
        write_param(ERROR_uC, 1);      // Écriture du code erreur = 1
        return;                        // Sortie immédiate de la routine
    }

    // Lecture de la consigne de température (exprimée en mV) 
    consigne = read_param(CONSIGNE);

    // Sélection du second convertisseur ADC (celui lié à la PT100)
    ADC_SetCS(ADC_SS_2);

    // Mesure de température lue sur le canal 2 avec moyenne glissante
    temp = ADC_readavrgChannel(ADC_CH2, moy_count_PT100);

    // Calcul de la commande PI en fonction de la consigne et de la mesure
    pwm_value =PI_Compute(consigne, temp);
    
    optical_system_enable(125);

    if(check_periode(read_param(Periode_Temp_PT100), &last_measure_temp)) {
        temp = ADC_readavrgChannel(ADC_CH2, read_param(moy_count_PT100));
        To_RAW(temp , 1); // température lue sur la PT100
    }

    if (check_periode(read_param(Periode_Courant), &last_measure_courant)) {
        //To_RAW(get_current(),DP_RAW_2);  //get_current()    
    }
 
    // Mise à jour des paramètres en RAM (ou vers d'autres modules)
    write_param(PWM_PI, pwm_value);                       // PWM calculé (0–255)
    write_param(PT100_1, temp);               // Valeur brute mesurée sur la PT100
    write_param(ERREUR_PI, PI_GetErreur());            // Erreur actuelle (consigne - mesure)
    write_param(SOMME_ERREUR_PI, PI_GetSomme_to_int16());     // Somme d’erreur filtrée (16 bits)

    // Génération du rapport cyclique pour PWM logiciel à 20 kHz
    ticks_haut = (pwm_value * periode_ticks) / 255;
    // "ticks_haut" : durée à l’état haut dans un cycle (rapport cyclique * période)
}

static void ScoopFSM_STABILISATION_Exit(ScoopFSM *fsm) {
    write_param(CTRL_uC, 0xe000);
    WRITE_GPIO1(0);
    desactivate_ISR_20kHz();
}

// ---- Etat SUPER_CRITICAL ----
static void ScoopFSM_SUPER_CRITICAL_Entry(ScoopFSM *fsm) {

    /*ADC_SetCS(ADC_SS_2);
    write_param(CTRL_uC, 0xe000); // activation des alimentations
    ina_init_gen(adrs_RC,  ina_config,  ina_calibration) ; // config du INA209 pour SCOOP adress 0x41 (DC)
    PWM_RC_parametres(read_param(CONSIGNE)); // config du PWM du rechauffeur
    activate_PWM_RC(); // activation du PWM du rechauffeur
    PWM_DC_parametres(read_param(CONSIGNE)); // config du PWM du doigt chauffant
    activate_PWM_DC(); // activation du PWM du doigt chauffant*/

    write_param(CTRL_uC, 0xe000); // activation des alimentations
    start_time = read_param(TIME_CCSD_MIN) * 60 + read_param(TIME_CCSD_100MS) / 10;
   
    activate_PWM_DC(); // Activation du PWM du doigt chauffant
    PWM_DC_parametres(0);

    Ecart_AD590=read_param(variation_AD590_Amplitude);
    Ecart_PT100=read_param(variation_PT100_Amplitude);
}

// ---- Routine exécutée périodiquement dans l’état SUPER_CRITCIAL ----
static void ScoopFSM_SUPER_CRITICAL_Routine(ScoopFSM *fsm) {
    uint16_t temp_PT100 ;
    uint16_t temp_AD590 ;
    uint16_t press;

    t_actuel = read_param(TIME_CCSD_MIN) * 60 + read_param(TIME_CCSD_100MS) / 10;
    if ((t_actuel - start_time) >= read_param(DUREE_MAX_SUPERCRITICAL)) {
        if (read_param(Temp_AD590) < 60) {
            
            // Régulation PI avec les AD590
                // Lecture de la consigne de température (exprimée en mV)
                onsigne =uint16_t(26.0f*read_param(CONSIGNE) + 1834.9f); // Conversion de la consigne en mV (les AD590);
                //consigne =uint16_t(201.43f*read_param(CONSIGNE) - 4198.0f); // Conversion de la consigne en mV (les PT100s) ;
                ADC_SetCS(ADC_SS_2);
                //temp = ADC_readavrgChannel(ADC_CH2, moy_count_PT100);
                temp_AD590 = ADC_readavrgChannel(ADC_CH0, moy_count_AD590);
                temp_PT100 = ADC_readavrgChannel(ADC_CH2, moy_count_PT100);
                press = ADC_readavrgChannel(ADC_CH4, moy_count_Pression);
                pwm_value = PI_Compute(consigne, temp_AD590);
                write_param(temp_AD590, AD590_1);               // Valeur brute mesurée sur la AD590_1
                write_param(ERREUR_PI, PI_GetErreur());            // Erreur actuelle (consigne - mesure)
                write_param(SOMME_ERREUR_PI, PI_GetSomme_to_int16());     // Somme d’erreur filtrée (16 bits)


                // Génération du rapport cyclique pour PWM logiciel à 20 kHz
                PWM_RC_parametres(pwm_value); // config du PWM du rechauffeur

            if(temp_AD590> max_AD590) max_AD590 = temp_AD590; 
            else min_AD590 = temp_AD590;
            if(temp_PT100> max_PT100) max_PT100 = temp_PT100; 
            else min_PT100 = temp_PT100;
            if(press> max_Pression) max_Pression = press;
            else min_Pression = press;
        
            // Vérification des écarts
            if ((max_AD590 - min_AD590) <= read_param(Ecart_AD590) &&
                (max_PT100 - min_PT100) <= read_param(Ecart_PT100) &&
                (max_Pression - min_Pression) <= read_param(variation_Pression)) {

                // Appliquer les variations de température via le doigt chauffant
                hundler_pwm_DC(read_param(T_ON_PWM), read_param(T_OFF_PWM), read_param(Rapport_Cyclique_DC));
                // Prendre les mesures souhaitées
                prendre_mesures(read_param(CAPTEUR_ENABLE));
            }
            else {
                write_param(Error_Code, (read_param(MODE) << 12) | 0x0001); // Ecart de température ou pression trop important
                fsm->nextState = ERROR;
                return;
            }

        } else {
            write_param(Error_Code, (read_param(MODE) << 12) | 0x0002); // Température meca (AD590) dépassée 60°C
            fsm->nextState = ERROR;
            return;
        }

    } else {
        write_param(Error_Code, (read_param(MODE) << 12) | 0x000D); // Durée max dépassée
        fsm->nextState = ERROR;
        return;
    }

    //verification de l'alimentation
    if ((read_param(CTRL_uC) & (MASK_CTRL_uC_3V3_S | MASK_CTRL_uC_5V_S | MASK_CTRL_uC_VBATT_S)) != 0xe000) {
        fsm->nextState = ERROR;
        write_param(Error_Code, (read_param(MODE) << 12) | 0x000A); // probleme d'alimentation 
        return;
    }



    /*// Mesure de température lue sur le canal 2 avec moyenne glissante
    write_param(RESERVE_DP_4, ADC_readSingleChannel(ADC_CH3)); // Valeur brute mesurée sur la PT100
    _delay_ms(100);
    write_param(RESERVE_DP_5, ADC_readSingleChannel(ADC_CH2)); // Valeur brute mesurée sur la PT100
    _delay_ms(100);

   write_param(RESERVE_DP_3,get_current()); // mesure du courant du rechauffeur
    _delay_ms(200);
    PWM_RC_parametres(read_param(CONSIGNE)); // config du PWM du rechauffeur
    _delay_ms(200);
    // Vérification de l'alimentation
    if((read_param(CTRL_uC) & (MASK_CTRL_uC_3V3_S | MASK_CTRL_uC_5V_S | MASK_CTRL_uC_VBATT_S)) != 0xe000){
        fsm->nextState = ERROR;
        write_param(Error_Code, 0x300A); // ERROR ID1 :  probleme d'alimentation 
    }*/

}

static void ScoopFSM_SUPER_CRITICAL_Exit(ScoopFSM *fsm) {
    desactivate_PWM_RC();
}

// ---- Etat RELEASE ----
static void ScoopFSM_RELEASE_Entry(ScoopFSM *fsm) {
    WRITE_GPIO1(0);
    desactivate_ISR_20kHz();
    optical_system_disable();
    write_param(CTRL_uC, 0x0000);
}

static void ScoopFSM_RELEASE_Routine(ScoopFSM *fsm) {
    
    _delay_ms(10);
}

static void ScoopFSM_RELEASE_Exit(ScoopFSM *fsm) {
    ;
}

// ---- Etat ERROR ----
static void ScoopFSM_Error_Entry(ScoopFSM *fsm) {
    WRITE_GPIO1(0);
    desactivate_PWM_DC();
    desactivate_PWM_RC();
    optical_system_disable();
    write_param(CTRL_uC, 0x0000);
}

static void ScoopFSM_Error_Routine(ScoopFSM *fsm) {
    _delay_ms(10);
}

static void ScoopFSM_Error_Exit(ScoopFSM *fsm) {
    ;
}

// ---- Etat END ----
static void ScoopFSM_End_Entry(ScoopFSM *fsm) {
    WRITE_GPIO1(0);
    desactivate_ISR_20kHz();
    write_param(CTRL_uC, 0x0000);
}

static void ScoopFSM_End_Routine(ScoopFSM *fsm) {
    _delay_ms(10);
}

static void ScoopFSM_End_Exit(ScoopFSM *fsm) {
    ;
}

//Config TIMER0 1ms entre deux Interruption  pour le PWM du doigt chauffant
void initTimer_DC()
{
    TCCR0 = 0;
    TCCR0 |= (1 << WGM01);          // Mode CTC
    TCCR0 |= (1 << CS01);           // Prescaler = 8 (CS02=0, CS01=1, CS00=0)
    OCR0 = 49;                      // TOP = 49 -> fréquence = 20 kHz
}

void activate_PWM_DC()
{
    TIMSK |= (1 << OCIE0);   // Enable Timer0 Compare Match Interrupt
}

void desactivate_PWM_DC()
{
    TIMSK &= ~(1 << OCIE0);  // enable interrupt
    WRITE_GPIO2(0);
}
// calcul des parametres du PWM doigt chauffant
void PWM_DC_parametres(uint16_t rapport_cyclique_pourcent)
{
    if(rapport_cyclique_pourcent >= 100) {
        count_pwm_DC = 0; // Reset the PWM counter
        ticks_haut_DC = 20; // Set ticks_haut_DC to 0 to disable PWM
    }

    else if(rapport_cyclique_pourcent < 5) {
        count_pwm_DC = 0; // Reset the PWM counter
        ticks_haut_DC = 0; // Set ticks_haut_DC to 0 to disable PWM
        WRITE_GPIO2(0); // Désactive le PWM du doigt chauffant
        }

    else {
    count_pwm_DC = 0; // Reset the PWM counter
    ticks_haut_DC = (uint16_t)( rapport_cyclique_pourcent * 0.2f);//(uint16_t)( (rapport_cyclique_pourcent / 100.0f) * periode_ticks_DC );
    }

}

void activate_PWM_RC()
{
    activate_ISR_20kHz();
}

void desactivate_PWM_RC()
{
    desactivate_ISR_20kHz();
    WRITE_GPIO1(0);
}

void PWM_RC_parametres(uint16_t rapport_cyclique_pourcent)
{
    if(rapport_cyclique_pourcent >= 100) {
        count_20k = 0; // Reset the PWM counter
        ticks_haut = 20; // Set ticks_haut_DC to 0 to disable PWM
    }

    else if(rapport_cyclique_pourcent < 5) {
        count_20k = 0; // Reset the PWM counter
        ticks_haut = 0; // Set ticks_haut_DC to 0 to disable PWM
        WRITE_GPIO2(0); // Désactive le PWM du doigt chauffant
        }

    else {
    count_20k = 0; // Reset the PWM counter
    ticks_haut = (uint16_t)( rapport_cyclique_pourcent * 0.2f);//(uint16_t)( (rapport_cyclique_pourcent / 100.0f) * periode_ticks_DC );
    }

}

int main(void) {
    initSpikCU(); //Init bus SPI
    specParameters(); //Chargement param specifique a SCOOP
    specChmod(get_chmod_ptr()); // chargement config param
    init_ADC(); // Init ADC MAX186
    initTimer_DC(); //Init Timer0 kHz pour le pwm du doigt chauffant
    initTimer20KHz(); //Init Timer3 20kHz
    INIT_GPIO1(OUT); // PWM réchauffeur 
    INIT_GPIO2(OUT); // PWM doigt chauffant 
    sei(); //

    ScoopFSM_init(&fsm); // Initialize the FSM

    while (1) {
        ScoopFSM_loop(&fsm); // Run the FSM loop
    }

    return 0;
}


// ---- Fonction shotHK ----
// Capture un "snapshot" des principales données de télémesure (Housekeeping)
// Les écritures sont faites dans un tampon ou un bus vers un système de télécommande/surveillance
void shotHK(void)
{
    // Écriture dans les emplacements de housekeeping :
    // 0 → État des sorties/commandes uC (alims, relais, etc.)
    write_HK(read_param(CTRL_uC), 0);

    // 1 → Mode actuel de la FSM
    write_HK(read_param(MODE), 1);

    // 2 → Prochain mode de la FSM (état suivant)
    write_HK(read_param(MODE_SUIVANT), 2);

    // 4 → Temps CCSDS exprimé en 100 ms (timestamp simplifié)
    write_HK(read_param(TIME_CCSDS_100MS), 4);

    // Si le système est dans un mode actif : mesure température à la volée
    if (read_param(MODE) == DIAG_Simple )
    {
       // write_HK(ADC_readavrgChannel(ADC_CH2, 1), 6);   6 → Mesure instantanée sur le canal ADC_CH2 (PT100_1)

        ADC_SetCS(ADC_SS_2);
        uint8_t captEnabled=read_param(CAPTEUR_ENABLE); 
        if(((captEnabled >>7 ) & 1 )==1)write_HK(ADC_readavrgChannel(ADC_CH2,read_param(moy_count_PT100)),5);
        if(((captEnabled >>6 ) & 1 )==1)write_HK(ADC_readavrgChannel(ADC_CH3,read_param(moy_count_PT100)),6);
        if(((captEnabled >>5 ) & 1 )==1)write_HK(ADC_readavrgChannel(ADC_CH0,read_param(moy_count_AD590)),7);
        if(((captEnabled >>4 ) & 1 )==1)write_HK(ADC_readavrgChannel(ADC_CH1,read_param(moy_count_AD590)),8);
        if(((captEnabled >>3 ) & 1 )==1)write_HK(ADC_readavrgChannel(ADC_CH4,read_param(moy_count_Pression)),9);
        if(((captEnabled >>2 ) & 1 )==1)write_HK(ADC_readavrgChannel(ADC_CH5,read_param(moy_count_OPT)),10);
    }

}

// ---- Interruption du Timer 3 : Compare Match A ----
// Déclenchée toutes les 50 µs (20 kHz) grâce au timer hardware configuré.
// Sert à générer un PWM logiciel sur une broche GPIO (ici GPIO1).  
ISR(TIMER3_COMPA_vect) {
    // Incrémentation du compteur interne à chaque appel de l'interruption
    count_20k++;

    // Vérifie si un cycle complet de PWM est terminé
    // "periode_ticks" correspond à la durée complète d’un cycle PWM (ex: 200 ticks pour 100 Hz)
    if (count_20k >= periode_ticks){ //periode_ticks=20 pour 1kHz
        count_20k = 0;  // Remise à zéro du compteur pour démarrer un nouveau cycle PWM
    }
    // Génération du rapport cyclique :
    // Si le compteur est inférieur ou égal à "ticks_haut", on met la sortie à 1 (état haut)
    // Sinon, on la met à 0 (état bas)
    //
    // "ticks_haut" est calculé dynamiquement comme :
    // ticks_haut = (pwm_value * periode_ticks) / 255;
    //
    // Cela permet d’ajuster le rapport cyclique entre 0 % (0) et 100 % (255) sur une période définie.
    if (count_20k < ticks_haut)
        WRITE_GPIO1(1);  // Active la broche de sortie (signal PWM haut)
    else 
        WRITE_GPIO1(0);  // Désactive la broche (signal PWM bas)
}

// ---- Interruption du Timer 0 : Compare Match A ----
// Déclenchée une interruption  chaque 20 KHz . 
ISR(TIMER0_COMP_vect) {

    count_pwm_DC++;
    if(count_pwm_DC >= periode_ticks) count_pwm_DC = 0;  //periode_ticks_DC =20 pour 1ms 1K du PWM 

    if(count_pwm_DC < ticks_haut_DC) WRITE_GPIO2(1);

    else WRITE_GPIO2(0);
    
}



