/*
   modifié le 08.04.2019
   par Anaelle Sarazin
   branchement des encodeurs
   changement de l'empatement + resolution pour encodeurs

*/


// /!\ Vmax = 200

#include <DueTimer.h>
#include <Servo.h>

//La table mesure 3000*2000mm
/*
   Conversions Utiles:
   mm->tic: nb_de_tic_a_parcourir = mm_a_parcourir * (double)ResEnc / (2 * PI * (double)RAYON_ROUE)
   degres -> tic : tic_a_parcourir = degres_a_parcourir * 2 * PI / 360.0 * (double)ResEnc * (double)EMPATEMENT / (2 * PI * (double)RAYON_ROUE));
*/


//*******************PARAMETRE GEOMETRIQUE*************************************//
#define RAYON_ROUE 41.6                  // Rayon des roues codeuses (en mm)
#define EMPATEMENT 337                   // Ecartement des roues codeuses (en mm)
//*****************************************************************************//


//*************************TIMERS***********************//
#define frequence_timer_asserv 50      // Frequence de calcul de l'odometrie et de l'assservissement (en Hz)
#define frequence_envoi_coord 10       // Frequence d'envoi des coordonnees du robot au master (en Hz)
#define tempsMatch 100                 // Temps du match (en secondes)
//*****************************************************//


//*************************ENCODEURS******************//
// Fil rouge : Motor+
// Fil noir : Motor-
// Fil bleu : encodeur VCC = 5V
// Fil vert : encodeur GND
// Fil jaune: encodeur piste A OUTavance
// Fil blanc: encodeur piste B OUT

#define EnDA 25        // Pin encodeur droit piste A (fil jaune moteur/noir encodeur)              
#define EnDB 24        // Pin encodeur droit piste B (fil blanc moteur/blanc encodeur)               
#define EnGA 22        // Pin encodeur gauche piste B (fil blanc moteur/noir encodeur)             
#define EnGB 23        // Pin encodeur gauche piste A (fil jaune moteur/blanc encodeur)

#define ResEnc 1024     // Resolution des encodeurs (en pas par tour), après réduction s'il y en a une           
//****************************************************//


//*************VARIABLE ODOMETRIE************************//
int encoderDPos = 0;                  // Position courante encodeur droit (en tic)
int encoderGPos = 0;                  // Position courante encodeur gauche (en tic)
int encoderDPos_old = 0;              // Position precedente encodeur droit (en tic)
int encoderGPos_old = 0;              // Position precedente encodeur gauche (en tic)

#define ODO_Theta_init 0           // Position angulaire initiale du robot (en radians)
#define ODO_X_init  1500              // Position initiale en X du robot (en mm) par rapport a l'origine de la table
#define ODO_Y_init  1000              // Position initiale en Y du robot (en mm) par rapport a l'origine de la table

int distance_parcourue_tic = 0;       // Distance parcourue (en tic) depuis la position d'origine
int orientation_tic = 0;              // Orientation angulaire (en tic)

double ODO_ds = 0.0;                  // Distance parcourue entre deux echantillonages (en mm)

double ODO_Theta = ODO_Theta_init;    // Position angulaire courante du robot (en radians)
double ODO_X = ODO_X_init;            // Position courante en X du robot (en mm) par rapport a l'origine de la table
double ODO_Y = ODO_Y_init;            // Position courante en Y du robot (en mm) par rapport a l'origine de la table

int ds_callage = 0;                   // Distance parcourue (en mm) entre la consigne de callage et le callage

long t;                               // Variable contenant du temps (en secondes) pour determiner si on doit afficher les coordonnes ou non
long t_old;
//*******************************************************//


//***************COMMANDE MOTEURS / CYTRON***************//
// /!\ pour moteur gauche inverser rouge et noir dans la nappe
#define PWMD 8    //pin pwm droit (fil orange)
#define DIRD 10   //pin dir droit 

#define PWMG 3    //pin pwm gauche (fil orange)
#define DIRG 5    //pin dir gauche
//*******************************************************//


//****************ASSERVISSEMENT************************//
int Vmax = 255;                        // Tension max commande moteur
int Vmin = 30;                         // Tension min commande moteur

double DkP = 2.5;                         // Gain proportionnel en distance
double DkD = 1.0;                         // Gain derive en distance
double OkP = 1.6;                         // Gain proportionnel en orientation
double OkD = 2.3;                         // Gain derive en orientation

int consigne_position_X = 0;              // Coordonnee en X de la consigne (en mm)
int consigne_position_Y = 0;              // Coordonnee en Y de la consigne  (en mm)

int consigne_distance = 0;                // Distance demandee en consigne  (en tic)
int consigne_orientation = 0;             // Orientation angulaire demandee en consigne (en degre)

int ThetaAbsC = 0;                        // Consigne (en degre) pour orientation absolue

int sensD = 0;                            // Sens de la distance a parcourir (-1 : en arriere ; 0 : a l'arret ; 1 : en avant)
double PID_consigne_distance = 0.0;       // Distance consigne de l'asservissement (en tic)
int sensO = 0;                            // Sens de l'orientation demandee
double PID_consigne_orientation = 0.0;    // Orientation consigne de l'asservissement (en tic)

double erreur_distance_old = 0;           // Erreur precedente en distance
double erreur_distance = 0;               // Erreur courante en distance
double derivee_distance = 0;              // Derivee de l'erreur en distance

double erreur_orientation_old = 0;        // Erreur precedente en orientation
double erreur_orientation = 0;            // Erreur courante en orientation
double derivee_orientation = 0;           // Derivee de l'erreur en orientation

int PWM_distance = 0;                     // Tension consigne de l'asservissement en distance
int PWM_orientation = 0;                  // Tension consigne de l'asservissement en orientation
int PWM_droit = 0;                        // Consigne de l'asservissement pour le moteur droit
int PWM_gauche = 0;                       // Consigne de l'asservissement pour le moteur gauche

#define cfcAvG 31                         // Pin capteur fin de course avant gauche
#define cfcAvD 30                         // Pin capteur fin de course avant droit
bool cfcAvGstate = false;                 // Etat du capteur fin de course avant gauche
bool cfcAvDstate = false;                 // Etat du capteur fin de course avant droit

#define cfcArG 48                         // Pin capteur fin de course arriere gauche
#define cfcArD 49                         // Pin capteur fin de course arriere droit
bool cfcArGstate = false;                 // Etat du capteur fin de course arriere gauche
bool cfcArDstate = false;                 // Etat du capteur fin de course arriere droit

int vitD_init;                             // Vitesse initiale du robot pendant un deplacement en distance (tic/temps de boucle)
int vitD;                                  // Vitesse courante du robot pendant un deplacement en distance (tic/temps de boucle)
int vitO_init;                             // Vitesse initiale du robot pendant un deplacement en orientation (tic/temps de boucle)
int vitO;                                  // Vitesse courante du robot pendant un deplacement en orientation (tic/temps de boucle)


bool go = false;                          // Marche (true) ; arret (false)
//********************************************************//


//************************EVITEMENT***********************//
int AVEUGLE = 1;            // Mode aveugle des US : pas d'evitement
int NORMAL = 2;             // Mode normal des US : arret complet quelque soit l'US qui detecte l'obstacle
int INTELLIGENT = 3;        // Mode intelligent des US : arret si l'US du sens de la marche detecte l'obstacle (si US arriere detecte lors d'une marche arriere ; si US avant détecte lors d'une marche avant)

int mode_US = AVEUGLE;       // Choix du mode d'utilisation des US (AVEUGLE ; NORMAL ; INTELLIGENT)

#define pin_AVD 32          // Pin de l'US avant droit
#define pin_AVG 34          // Pin de l'US avant gauche
#define pin_AR 35           // Pin de l'US arriere

boolean state_AVD = 0;      // Etat de l'US avant droit
boolean state_AVG = 0;      // Etat de l'US avant gauche
boolean state_AR = 0;       // Etat de l'US arriere
//********************************************************//


//*******************GESTION IA***************************//
#define LED_BLEU 52             // Pin LED bleue
#define SWITCH_IA 53            // Pin interrupteur slave/master
#define SWITCH_COULEUR 50       // Pin interrupteur orange/vert
#define SWITCH_TIRETTE 40       // Pin capteur fin de course de la tirette

bool SLAVE = true;              // Mode slave du robot : le robot prend les consignes via l'interface
bool MASTER = false;            // Mode master du robot : le robot execute le parcours decrit dans le code
bool IA_STATE = MASTER;         // Mode d'utilisation du robot (SLAVE ; MASTER)

bool ORANGE = 0;                // Parcours equipe orange
bool VERT  = 1;                 // Parcours equipe verte
bool couleur;                   // Parcours de la couleur de l'equipe du robot

#define ledR 2                  // Pin R de la LED RVB
#define ledV 11                 // Pin V de la LED RVB
#define ledB 4                  // Pin B de la LED RVB
//****************************************************************//


//************************ACTIONNEURS******************************//
//**********ATTRIBUTION DES PINS*******************
#define CBG 11                              //pin commande servo bras gauche 
#define CBD 9                               //pin commande servo bras droit 
#define CR 12                               //pin commande servo râteau
#define CP 13                               //pin commande servo pont levis

//*********DEFINITION DES SERVOS*******************
Servo Servo_Bras_G;                         //création d'un objet servo pour contrôler un servo
Servo Servo_Bras_D;
Servo Servo_Rateau;
Servo Servo_Pont;
//****************************************************************//


//**************COMMUNICATION************************************//
int valeur = 0;                                     // Prend une valeur (correspondant à une action a executer) d'apres un type de consigne demandee via l'interface
int  nombreRecu = 0;                                // Prend la valeur numerique de la consigne demandee via l'interface

#define TAILLE_MAX 1000                             // Taille du tableau de donnees renvoyees

// Tableaux de donnees pour faire des graphes sur matlab:
int consigne__distance[TAILLE_MAX];
int consigne__orientation[TAILLE_MAX];
int ODO__X[TAILLE_MAX];
int ODO__Y[TAILLE_MAX];
int ODO__Theta[TAILLE_MAX];
int PID__distance[TAILLE_MAX];
int PID__orientation[TAILLE_MAX];
int PWM__droite[TAILLE_MAX];
int PWM__gauche[TAILLE_MAX];
int distance_[TAILLE_MAX];

int j = 0;                                          // Variable iterative pour l'envoi des donnees
//***************************************************************//


//*************PROTOTYPES DES FONCTIONS****************************//
//Comptent les fronts montants et descendants des encodeurs (interruptions sur pin):
void GestionInterruptionCodeurDA();         // Encodeur droit piste A
void GestionInterruptionCodeurDB();         // Encodeur droit piste B
void GestionInterruptionCodeurGA();         // Encodeur gauche piste A
void GestionInterruptionCodeurGB();         // Encodeur gauche piste B

void routineDeplacement();                  // Execute les fonctions odometrie, Asservissement, moteurD, et moteurG à 100Hz

void odometrie();                           // Calcul la position du robot

void envoi_coordonnees();                   // Envoi les coordonnees du robot à l'interface

void moteurD(int mli);                      // Commande en pwm le moteur droit
void moteurG(int mli);                      // Commande en pwm le moteur gauche

void avance(int mm);                        // Fait avancer le robot de la distance (en mm) en argument
void tourne(double degres);                 // Fait tourner le robot de l'angle (en degres) en argument

void aimShoot(int x, int y);                // Deplace le robot jusqu'aux coordonnees en argument

void orientationAbsolue(double degres);     // Oriente le robot d'un angle (en degres) absolu par rapport a l'orientation initiale

void Asservissement();                      // Asservit le robot

void callageAV(int timeout = 1000);         // Realise un callage avant
void callageAR(int timeout = 1000);         // Realise un callage arriere

void assignation_donnee();                  // Assigne une action a realiser en fonction de la variable valeur (d'apres le type de consigne demandee via l'interface)
void reception_donnees();                   // Affecte un entier a la variable valeur d'apres le type de consigne demandee via l'interface

void affiche_coeff();                       // Affiche les gains des differents asservissemens, et des vitesses min et max

void setODOX(int x);                        // Affecte a la variable ODO_X la valeur entree en argument (en mm)
void setODOY(int y);                        // Affecte a la variable ODO_Y la valeur entree en argument (en mm)
void setODOTheta(int angle);                // Affecte a la variable ODO_Theta la valeur entree en argument (en degres)

void ledverte();                            // Allume la LED RVB en vert
void ledorange();                           // Allume la LED RVB en orange
void ledrose();                             // Allume la LED RVB en rose
void ledeteinte();                          // Eteint la LED RVB

void tirette();                             // Affecte la couleur du match tant que la tirette est presente
void finmatch();                            // Arrete le robot au bout du temps de match reglementaire, fait clignoter la LED bleue, et fait clignoter la LED RVB en rose
void evitement();                           // Realise le mode d'evitement choisi : aveugle, normal, ou intelligent


//FONCTIONS ACTIONNEURS
void Controle_Bras(String deplacement);    //"ouverture" ou "fermeture" des bras
void Controle_Rateau(String deplacement);  //"descente" ou "montee" du râteau
void Controle_Pont(String deplacement);    //"descente" ou "montee" du pont levis
//***************************************************************//



void setup() {

  Serial.begin(115200);                                             // Transmission a 115 200 octets/seconde

  // Encodeurs (les pullup internes à la carte sont obligatoires) :
  pinMode(EnDA, INPUT_PULLUP);                                      // Encodeur droit piste A
  pinMode(EnDB, INPUT_PULLUP);                                      // Encodeur droit piste B
  pinMode(EnGA, INPUT_PULLUP);                                      // Encodeur gauche piste A
  pinMode(EnGB, INPUT_PULLUP);                                      // Encodeur gauche piste B

  // Interruption sur pin liées aux encodeurs :
  attachInterrupt(EnDA, GestionInterruptionCodeurDA, CHANGE);       // Encodeur droit piste A
  attachInterrupt(EnDB, GestionInterruptionCodeurDB, CHANGE);       // Encodeur droit piste B
  attachInterrupt(EnGA, GestionInterruptionCodeurGA, CHANGE);       // Encodeur gauche piste A
  attachInterrupt(EnGB, GestionInterruptionCodeurGB, CHANGE);       // Encodeur gauche piste B

  // Commande moteur :
  pinMode(PWMD, OUTPUT);                                            //pwm moteur droit
  pinMode(DIRD, OUTPUT);                                            //dir moteur droit
  pinMode(PWMG, OUTPUT);                                            //pwm moteur gauche
  pinMode(DIRG, OUTPUT);                                            //dir moteur gauche

  // US :
  pinMode(pin_AVD, INPUT);                                          // US avant droit
  pinMode(pin_AR, INPUT);                                           // US avant arriere
  pinMode(pin_AVG, INPUT);                                          // US avant gauche

  // Capteurs fin de course :
  pinMode(cfcAvG, INPUT_PULLUP);                                    // Capteur fin de course avant gauche
  pinMode(cfcAvD, INPUT_PULLUP);                                    // Capteur fin de course avant droit
  pinMode(cfcArG, INPUT_PULLUP);                                    // Capteur fin de course arriere gauche
  pinMode(cfcArD, INPUT_PULLUP);                                    // Capteur fin de course arriere droit

  //IA :
  pinMode(LED_BLEU, OUTPUT);                                        // LED bleue
  digitalWrite(LED_BLEU, LOW);                                      // LED bleue initialement eteinte
  pinMode(SWITCH_IA, INPUT_PULLUP);                                 // Interrupteur slave/master
  pinMode(SWITCH_TIRETTE, INPUT_PULLUP);                            // Capteur fin de course de la tirette
  pinMode(SWITCH_COULEUR, INPUT_PULLUP);                            // Interrupteur orange/vert
  IA_STATE = digitalRead(SWITCH_IA);                                // Initialisation du mode d'utilisation du robot (slave ou master)
  pinMode(ledR, OUTPUT);                                            // R, LED RVB
  pinMode(ledV, OUTPUT);                                            // V, LED RVB
  pinMode(ledB, OUTPUT);                                            // B, LED RVB

  t = millis();                                                     // Initialisation de compteur de temps
  t_old = millis();

  Timer7.attachInterrupt(finmatch);                                 // Timer7 pour execution de la fonction finmatch

  Timer1.attachInterrupt(routineDeplacement);                       // Timer1 pour execution de la fonction routineDeplacement
  Timer1.setFrequency(frequence_timer_asserv);                      // Timer1 a la frequence de frequence_timer_asserv
  Timer1.start();                                                   // Demarrage Timer1

  //servomoteurs
  Servo_Bras_G.attach(CBG);                 //attribue la bonne pin de commande au bon servo
  Servo_Bras_D.attach(CBD);
  Servo_Rateau.attach(CR);
  Servo_Pont.attach(CP);

  //initialisation des positions des servos
  //(NE PAS MODIFIER)
  Servo_Bras_D.write(180);                  //repos: 180 degrés (ces 2 servos doivent tourner en sens inverse, faire un schéma si besoin)
  Servo_Bras_G.write(0);                    //repos: 0 degrés
  Servo_Rateau.write(0);
  Servo_Pont.write(0);


  if (IA_STATE == SLAVE) {                                          // Si utilisation du robot en mode slave, LED bleue allumee et LED RVB eteinte
    digitalWrite(LED_BLEU, HIGH);
    ledeteinte();
  }
  else {                                                            // Si utilisation du robot en mode master, LED bleue eteinte, execution de la fonction tirette, et demarrage du Timer7
    digitalWrite(LED_BLEU, LOW);
    tirette();
    Timer7.start(tempsMatch * 1000 * 1000);
  }

  Serial.println("debut loop");
}



void loop() {
  //**********************MODE D'UTILISATION DU ROBOT ET PARCOURS**********************//

//  while (1)
//  { // ... Faire indefiniment...
//    // Description du parcours test :
//
//          // Se mettre en position de depart :
//          pause(2000);
//          tourne(-90);
//          callageAR();
//
//          // Aller chercher les palets de la zone chaos :
//          avance(890);
//          pause(1000);
//          tourne(90);
//          pause(1000);
//          Controle_Bras("ouverture");
//          avance(430);
//          Controle_Bras("attraper");
//          pause(2000);
//
//          // Revenir deposer les palets dans la zone rouge :
//          tourne(145);
//          avance(890+90);
//          pause(1000);
//          tourne(180-145);
//          pause(1000);
//          Controle_Bras("relacher");
//          avance(-700+90);
//          pause(1000);
//          Controle_Bras("fermeture");
//
//          // Aller chercher les palets sur le petit rack :
//          tourne(90);
//          pause(1000);
//          avance(-300);
//          callageAR();
//          pause(1000);
//          avance(80);
//          pause(1000);
//          tourne(90);
//          pause(1000);
//          avance(810-90);
//          pause(3000); //actions sur le rack
//
//          // Aller deposer les palets dans la balance :
//          tourne(-90);
//          pause(1000);
//          avance(500); 
//
//    while (1); //fin du programme
//
//  }

//      while (1) {                             // ... Faire indefiniment...
//        avance(500);
//        tourne(95);
//       }

      while (1) {                             // ... Faire indefiniment...
        envoi_coordonnees();                     // Envoi des coordonnees a l'interface
         assignation_donnee();                    // Assignation de l'action a realiser en fonction de la variable valeur (d'apres le type de consigne demandee via l'interface)
         reception_donnees();                     // Affecte un entier a la variable valeur d'apres le type de consigne demandee via l'interface
       }
}

//****************************DEFINITION DES FONCTIONS****************************//

// COMPTEURS FRONTS MONTANTS ET DESCENDANTS DES ENCODEURS : (voir chronogrammes encodeurs pour le fonctionnement)
// Encodeur droit piste A
void GestionInterruptionCodeurDA()                         // Lance si piste A codeur droit change d'etat
{
  if (digitalRead(EnDA) == digitalRead(EnDB)) {
    encoderDPos--;
  }
  else {
    encoderDPos++;
  }
}

// Encodeur droit piste B
void GestionInterruptionCodeurDB()                         // Lance si piste B codeur droit change d'etat
{
  if (digitalRead(EnDA) == digitalRead(EnDB)) {
    encoderDPos++;
  }
  else {
    encoderDPos--;
  }
}

// Encodeur gauche piste A
void GestionInterruptionCodeurGA()                         // Lance si piste A codeur gauche change d'etat
{
  if (digitalRead(EnGA) == digitalRead(EnGB)) {
    encoderGPos--;
  }
  else {
    encoderGPos++;
  }
}

// Encodeur gauche piste B
void GestionInterruptionCodeurGB()                         // Lance si piste B codeur gauche change d'etat
{
  if (digitalRead(EnGA) == digitalRead(EnGB)) {
    encoderGPos++;
  }
  else {
    encoderGPos--;
  }
}



// ROUTINE DE NAVIGATION :
void routineDeplacement() {
  odometrie();                    // Calcul de la position du robot
  if (go == true) {               // Si robot en marche...
    evitement();                    // Evitement par capteurs US
    Asservissement();               // Asservissement robot
    j++;                            // Compteur pour le stockage des donnees
  }
  moteurG(PWM_gauche);            // Envoi de la consigne en PWM au moteur gauche
  moteurD(PWM_droit);             // Envoi de la consigne en PWM au moteur droit
}



// CALCUL DE LA POSITION DU ROBOT :
void odometrie() {
  distance_parcourue_tic = (encoderDPos + encoderGPos) / 2.0;     // Calcul de la distance parcourue en tic = moyenne des positions courantes en tic des encodeurs droit et gauche
  orientation_tic = encoderDPos - encoderGPos;                    // Calcul de l'orientation en tic = difference entre les positions courantes en tic des encodeurs droit et gauche

  // Calcul du nombre de tics faits depuis le dernier passage dans la fonction (vitesse en nombre de tics par temps d'echantillonge) :
  int delta_tic_d = encoderDPos - encoderDPos_old;                // Difference entre les positions en tic courante et precedente sur encodeur droit
  encoderDPos_old = encoderDPos;                                  // Stockage de la position courante dans la position precedente sur encodeur droit
  int delta_tic_g = encoderGPos - encoderGPos_old;                // Difference entre les positions en tic courante et precedente sur encodeur gauche
  encoderGPos_old = encoderGPos;                                  // Stockage de la position courante dans la position precedente sur encodeur gauche

  ds_callage += (delta_tic_d + delta_tic_g) / 2.0;                // Calcul de la distance parcourue entre la consigne de callage et le callage = ds_callage precedent + moyenne des delta_tic droit et gauche

  double delta_d = delta_tic_d * 2.0 * PI / (double)ResEnc;               // Calcul de la variation de position angulaire de la roue droite (en radian)
  double delta_g = delta_tic_g * 2.0 * PI / (double)ResEnc;               // Calcul de la variation de position angulaire de la roue gauche (en radian)

  ODO_ds = (double)RAYON_ROUE * (delta_g  + delta_d) / 2.0  ;             // Calcul du petit deplacement (en mm) elementaire du robot (distance parcourue entre deux echantillonages)

  double dtheta = (double)RAYON_ROUE / (double)EMPATEMENT * (delta_d - delta_g); // Calcul de la variation d'orientation (en radian) du robot

  ODO_Theta += dtheta ;                                           // Calcul de la position angulaire du robot (en radian)

  // Calcul de la position angulaire entre -pi et pi :
  if (ODO_Theta > PI) {                                           // Si position angulaire du robot superieure a pi...
    ODO_Theta = ODO_Theta - 2 * PI;                                   // Position diminuee de 2 pi
  }
  else if (ODO_Theta < -PI) {                                     // Si position angulaire du robot inferieure a -pi...
    ODO_Theta = ODO_Theta + 2 * PI;                                   // Position augmentee de 2 pi
  }

  // Calcul la position du robot :
  ODO_X = ODO_X + ODO_ds * cos(ODO_Theta);                        // Coordonnee en X courante (en mm) = coordonnee en X precedente + distance elementaire * cosinus de la position angulaire
  ODO_Y = ODO_Y + ODO_ds * sin(ODO_Theta);                        // Coordonnee en Y courante (en mm) = coordonnee en Y precedente + distance elementaire * sinus de la position angulaire

  // Calcul des tableaux pour matlab :
  if (go == true && j < TAILLE_MAX) {
    ODO__X[j] = (int)ODO_X;
    ODO__Y[j] = (int)ODO_Y;
    ODO__Theta[j] = (int)(ODO_Theta * 360 / 2.0 / PI * 100);
    distance_[j] = distance_parcourue_tic;
  }

}



// ENVOI DES COORDONNEES DU ROBOT A L'INTERFACE :
void envoi_coordonnees() {
  t = millis();                                                               // t devient la duree ecoulee depuis le demarrage de l'execution du programme (en milli secondes)
  if ((t - t_old) > (1 / (double)(frequence_envoi_coord) * 1000)) {           // Si la duree ecoulee depuis le dernier passage dans la fonction (t - t_old) est superieur a la periode d'envoi des coordonnees...
    t_old = t;                                                                    // Stockage de t dans t_old

    Serial.print("X "); Serial.println((int)ODO_X);                               // Envoi des coordonnees en X (en mm)
    Serial.print("Y "); Serial.println((int)ODO_Y);                               // Envoi des coordonnees en Y (en mm)
    Serial.print("A "); Serial.println((int)(ODO_Theta * 360 / (2 * PI)));        // Envoi de la position angulaire (en degre)



    // Serial.println(encoderDPos);
    // Serial.println(encoderGPos);

    // Serial.println(orientation_tic);


    // Serial.print(encoderDPos_old);
    // Serial.print("  ");
    // Serial.println(encoderDPos);

    //Serial.println(dtheta);
  }
}



// COMMANDES MOTEURS :
// Moteur droit
void moteurD(int mli) {
  if (mli >= 0) {                 // Si marche avant (consigne positive)...
    digitalWrite(DIRD, LOW);            // Pas d'alimentation DIRD
    analogWrite(PWMD, mli);           // Envoi de la consigne pwm au moteur
  }
  else {                          // Si marche arriere (consigne negative)...
    digitalWrite(DIRD, HIGH);           // Alimentation DIRD
    analogWrite(PWMD, -mli);          // Envoi de l'oppose de la consigne pwm au moteur
  }
}

// Moteur gauche
void moteurG(int mli) {
  if (mli >= 0) {                 // Si marche avant (consigne positive)...
    digitalWrite(DIRG, HIGH);           // Alimentation DIRG
    analogWrite(PWMG, mli);           // Envoi de la consigne pwm au moteur
  }
  else {                          // Si marche arriere (consigne negative)...
    digitalWrite(DIRG, LOW);            // Pas d'alimentation DIRG
    analogWrite(PWMG, -mli);          // Envoi de l'oppose de la consigne pwm au moteur
  }
}



// FAIRE UNE PAUSE :
void pause(int ms) {                      // Pause d'une duree de l'argument (en millisecondes). Permet de ne pas utiliser delay, problematique avec interruptions
  long t_init = millis();                 // t_init prend la valeur depuis le demarrage de l'execution du programme
  long t_courant = millis();              // t_courant prend la valeur depuis le demarrage de l'execution du programme
  while (t_courant - t_init < ms) {       // Tant que le temps ecoule depuis l'entree dans la fonction est inferieur a la duree de pause souhaitee...
    t_courant = millis();                       // t_courant prend la valeur depuis le demarrage de l'execution du programme
    delayMicroseconds(10);                      // Attente de 10 microsecondes
  }
}



// RECEPTION DES DONNEES DEPUIS L'INTERFACE :
void reception_donnees()
{
  int octetRecu = 0;                   // Variable de l'octet recu (caracterise le type d'action demandee via l'interface)
  boolean signe = true;                // Variable du signe nombre recu (true : + ; false : -)
  valeur = 0;                          // Un entier lui sera affecte selon l'octet recu (caracterise le type d'action a executer via le programme)
  nombreRecu = 0;                      // Variable du nombre recu

  while (Serial.available() > 0) {     // Tant qu'un octet est recu...
    octetRecu = Serial.read();            // Lecture du premier octet recu et affectation de valeur selon octet_recu :
    if (octetRecu == 'a') {
      valeur = 0;                             // Ne rien faire
    }
    else if (octetRecu == 'b') {
      valeur = 1;                             // Redefinition du gain proportionnel en distance  DkP * 1000
    }
    else if (octetRecu == 'c') {
      valeur = 2;                             // Redefinition du gain derive en distance DkD * 1000
    }
    else if (octetRecu == 'd') {
      valeur = 3;                             // Redefinition du gain proportionnel en orientation  OkP * 1000
    }
    else if (octetRecu == 'e') {
      valeur = 4;                             // Redefinition du gain derive en orientation  OkD * 1000
    }
    else if (octetRecu == 'f') {
      valeur = 5;                             // Redefinition de la coordonnee en X (en mm)(fonction SetODO_X)
    }
    else if (octetRecu == 'g') {
      valeur = 6;                             // Redefinition de la coordonnee en Y (en mm)(fonction SetODO_Y)
    }
    else if (octetRecu == 'h') {
      valeur = 7;                             // Redefinition de la position angulaire (en radian)(fonction SetODO_Theta)
    }
    else if (octetRecu == 'i') {
      valeur = 8;                             // Ne rien faire
    }
    else if (octetRecu == 'j') {
      valeur = 9;                             // Ne rien faire
    }
    else if (octetRecu == 'k') {
      valeur = 10;                            // Ne rien faire
    }
    else if (octetRecu == 'l') {
      valeur = 11;                            // Ne rien faire
    }
    else if (octetRecu == 'm') {
      valeur = 12;                            // Ne rien faire
    }
    else if (octetRecu == 'n') {
      valeur = 13;                            // Redefinition de la tension max moteur Vmax
    }
    else if (octetRecu == 'o') {
      valeur = 14;                            // Redefinition de la tension min moteur Vmin
    }
    else if (octetRecu == 'p') {
      valeur = 15;                            // Ne rien faire
    }
    else if (octetRecu == 'q') {
      affiche_coeff();                        // Affichage des gains des differents asservissements, valeurs de Vmin et Vmax
    }
    else if (octetRecu == 'r') {
      //      reset();                        // Annule la consigne
    }
    else if (octetRecu == 's') {
      //      arret_urgence();                // Arret d'urgence programme sur robot
    }
    else if (octetRecu == 'X') {
      valeur = 20;                            // Consigne de coordonnee en X (en mm)
    }
    else if (octetRecu == 'Y') {
      valeur = 21;                            // Consigne de coordonnee en Y (en mm)
    }
    else if (octetRecu == 'A') {
      valeur = 22;                            // Consigne de distance a parcourir (en mm)(fonction avance)
    }
    else if (octetRecu == 'B') {
      valeur = 23;                            // Consigne d'orientation (en radian)(fonction tourne)
    }
    else if (octetRecu == 'C') {
      valeur = 24;                            // Consigne de position angulaire absolue (en degre)
    }
    else if (octetRecu == 'T') {
      valeur = 25;                            // Envoi les donnees a matlab (fonction EnvoyerData)
    }
    else if (octetRecu == 'L') {
      valeur = 26;                            // Ne rien faire
    }
    else if (octetRecu == 'M') {
      valeur = 27;                            // Ne rien faire
    }
    else if (octetRecu == 'N') {
      valeur = 28;                            // Callage avant (fonction callageAV)
    }
    else if (octetRecu == 'O') {
      valeur = 29;                            // Callage arriere (fonction callageAR)
    }
    else if (octetRecu == 'G')
    {
      //      mode_deplacement = 1; go = true; j = 0; freinD = 0; freinO = 0;         // Consigne de position en coordonnees polaires
      //      arret = false;
    }
    else if (octetRecu == 'P')
    {
      valeur = 30;                            // Consigne de position (coordonnees en mm)(fonction aimShoot)
    }
    else if (octetRecu == '-') {
      signe = false;                          // Si premier octet recu est ' - ', nombre negatif
    }
    else {
      octetRecu = octetRecu - 48;             // Conversion de la valeur ASCII recue en valeur decimale
      if ((octetRecu >= 0) && (octetRecu <= 9)) {       // Si valeur decimale recue comprise entre 0 et 9...
        nombreRecu = (nombreRecu * 10) + octetRecu;         // Conversion du nombre recue en valeur decimale
      }
    }
    delayMicroseconds(1000);                  // Pause de 1 seconde pour laisser le temps a la fonction available de recevoir l'octet suivant
  }

  if (signe == false) {                       // Si nombre négatif...
    nombreRecu = -nombreRecu;                     // Prise en compte du signe négatif
  }
}




// ASSIGNATION D'UNE ACTION A EXECUTER APRES RECEPTION DES DONNEES ENVOYEES PAR L'INTERFACE :
void assignation_donnee() {
  if (valeur == 1) {
    DkP = (double)(nombreRecu) / 1000.0;                            // Redefinition du gain proportionnel en distance  DkP * 1000
    Serial.print("c "); Serial.println(DkP * 1000);
  }
  else if (valeur == 2) {
    DkD = (double)(nombreRecu) / 1000.0;                            // Redefinition du gain derive en distance  DkD * 1000
    Serial.print("c "); Serial.println(DkD * 1000);
  }
  else if (valeur == 3) {
    OkP = (double)(nombreRecu) / 1000.0;                            // Redefinition du gain proportionnel en orientation  OkP * 1000
    Serial.print("c "); Serial.println(OkP * 1000);
  }
  else if (valeur == 4) {
    OkD = (double)(nombreRecu) / 1000.0;                            // Redefinition du gain derive en orientation  OkD * 1000
    Serial.print("c "); Serial.println(OkD * 1000);
  }
  else if (valeur == 5) {
    Serial.print("c "); Serial.println(nombreRecu);
    setODOX(nombreRecu);                                            // Redefinition de la coordonnee en X (en mm)
  }
  else if (valeur == 6) {
    Serial.print("c "); Serial.println(nombreRecu);
    setODOY(nombreRecu);                                            // Redefinition de la coordonnee en Y (en mm)
  }
  else if (valeur == 7) {
    Serial.print("c "); Serial.println(nombreRecu);
    setODOTheta(nombreRecu);                       // Redefinition de la position angulaire (en radian)
  }
  else if (valeur == 8) {}                                          // Ne rien faire
  else if (valeur == 9) {}                                          // Ne rien faire
  else if (valeur == 10) {}                                         // Ne rien faire
  else if (valeur == 11) {}                                         // Ne rien faire
  else if (valeur == 12) {}                                         // Ne rien faire
  else if (valeur == 13) {
    Vmax = nombreRecu;                                              // Redefinition de la tension max moteur Vmax
    Serial.print("c "); Serial.println(Vmax);
  }
  else if (valeur == 14) {
    Vmin = nombreRecu;                                              // Redefinition de la tension min moteur Vmin
    Serial.print("c "); Serial.println(Vmin);
  }
  else if (valeur == 15) {}                                         // Ne rien faire
  else if (valeur == 20) {
    consigne_position_X = (int)(nombreRecu);                        // Consigne de position en X (en mm)
    Serial.print("c X"); Serial.println(consigne_position_X);
  }
  else if (valeur == 21) {
    consigne_position_Y = (int)(nombreRecu);                        // Consigne de position en Y (en mm)
    Serial.print("c Y"); Serial.println(consigne_position_Y);
  }
  else if (valeur == 22) {
    Serial.print("c "); Serial.println(nombreRecu);
    avance(nombreRecu);                                             // Consigne de distance a parcourir (en mm)
  }
  else if (valeur == 23) {
    Serial.print("c "); Serial.println(nombreRecu);
    tourne(nombreRecu);                                             // Consigne d'orientation (en radian)
  }
  else if (valeur == 24) {
    ThetaAbsC = nombreRecu ;                                        // Consigne de position angulaire (en degre)
    // Serial.print("c "); Serial.println(nombreRecu);
    orientationAbsolue(ThetaAbsC);
  }
  else if (valeur == 25) {
    Serial.print("c "); Serial.println("envoye");
    EnvoyerData();                                                  // Envoi les donnees a matlab
  }
  else if (valeur == 26) {}                                         // Ne rien faire
  else if (valeur == 27) {}                                         // Ne rien faire
  else if (valeur == 28) {
    Serial.print("c "); Serial.println("Callage AV recu");
    callageAV();                                                    // Callage avant
  }
  else if (valeur == 29) {
    Serial.print("c "); Serial.println("Callage AR recu");
    callageAR();                                                    // Callage arriere
  }
  else if (valeur == 30) {
    Serial.print("c "); Serial.println("Aim&Shoot");
    aimShoot(consigne_position_X, consigne_position_Y);             // Consigne de position (coordonnees en mm)
  }

  valeur = 0;                                                       // Reinitialisation de la variable valeur
}



// AFFICHAGE DE COEFFICIENTS (lettre de commande 'q'):
void affiche_coeff() {
  Timer1.stop();                          // Arret du Timer1

  Serial.print("DkP ");
  Serial.println((int)(DkP * 1000));      // Affichage du gain proportionnel en distance

  Serial.print("DkD ");
  Serial.println((int)(DkD * 1000));      // Affichage du gain derive en distance

  Serial.print("OkP ");
  Serial.println((int)(OkP * 1000));      // Affichage du gain proportionnel en orientation

  Serial.print("OkD ");
  Serial.println((int)(OkD * 1000));      // Affichage du gain derive en orientation

  Serial.print("Vmax ");
  Serial.println(Vmax);                   // Affichage de la tension max moteur

  Serial.print("Vmin ");
  Serial.println(Vmin);                   // Affichage de la tension min moteur

  Timer1.start();                         // Redemarrage du Timer1
}




// ENVOI DES DONNEES A MATLAB (lettre de commande 'T'):
void EnvoyerData() {
  Timer1.stop();                                        // Arret du Timer1

  for ( int i = 0 ; i < TAILLE_MAX ; i++)
  {
    Serial.print("d ");
    Serial.print(i);
    Serial.print(" ");
    Serial.print(PID__distance[i]);
    Serial.print(" ");
    Serial.print(PID__orientation[i]);
    Serial.print(" ");
    Serial.print(PWM__droite[i]);
    Serial.print(" ");
    Serial.print(consigne__distance[i]);
    Serial.print(" ");
    Serial.print(ODO__X[i]);
    Serial.print(" ");
    Serial.print(ODO__Y[i]);
    Serial.print(" ");
    Serial.print(ODO__Theta[i]);
    Serial.print(" ");
    Serial.print(distance_[i]);
    Serial.print(" ");
    Serial.println(consigne__orientation[i]);            // Println pour fermer la phrase
  }
  Serial.println();

  Timer1.start();                                        // Redemarrage du Timer1
}




// GENERATION D'UN CONSIGNE DE DISTANCE :
void avance(int mm) {
  /*
     Génaration d'une consigne de distance, donnee en argument (en mm), avec un trapeze de vitesse
     PID_consigne_distance : consigne en distance envoyee a l'asservissement
     consigne_distance : distance a atteindre
  */
  // Initialisation des variables
  boolean freinD = false;                   // Frein en direction (false : pas de ralentissement ; true : ralentissement)
  vitD_init = 45;                           // Vitesse maximale en direction du robot (en tic / temps de boucle) initialisee
  vitD = vitD_init;                         // Vitesse courante du robot (en tic / temps de boucle)
  int Ad = 1;                               // Consigne d'acceleration du robot (en tic / temps de boucle²) initialisee
  int Dfd = 0;                              // Distance de freinage necessaire a l'arret du robot (en tic)
  int Vc = 0;                               // Consigne de vitesse du robot (en tic / temps de boucle)
  bool finDeplacementDistance = false;      // Etat du deplacement (false : robot en deplacement ; true : deplacement fini)

  // Calcul de la consigne
  consigne_distance = consigne_distance + (int)(mm * (double)ResEnc / (2 * PI * (double)RAYON_ROUE));         // Consigne courante (en tic) = consigne precedente (en tic) + distance demandee en argument de la fonction (convertie en tic)
  go = true;                                                                                  // Mise en marche du robot
  if (consigne_distance - PID_consigne_distance >= 0) {                                       // Si
    sensD = 1;                                                                                      // Marche avant
  } else {                                                                                    // Sinon...
    sensD = -1;                                                                                     // Marche arriere
  }

  // Generation du trapeze de vitesse
  while (finDeplacementDistance == false) {                                       // Tant que le deplacement n'est pas fini...
    if (freinD == false) {                                                                                                                    // Si pas en ralentissement...
      Dfd = 0.53 * Vc * Vc / Ad;                                                                                                                    // Calcul de la distance de freinage (en tic) pour la vitesse de consigne (le 0.55 sert a empecher les bugs dus a la discretisation)
    }
    if ((consigne_distance - PID_consigne_distance) * (consigne_distance - PID_consigne_distance) > Dfd * Dfd && Vc * Vc < vitD * vitD) {     // Phase d'acceleration
      PID_consigne_distance = PID_consigne_distance + Vc;
      Vc = Vc + Ad * sensD;
    }
    else if ((consigne_distance - PID_consigne_distance) * (consigne_distance - PID_consigne_distance) > Dfd * Dfd && Vc * Vc >= vitD * vitD) // Phase a vitesse constante
    {
      Vc = vitD * sensD;
      PID_consigne_distance = PID_consigne_distance + Vc;
    }
    else if ((consigne_distance - PID_consigne_distance) * (consigne_distance - PID_consigne_distance) <= Dfd * Dfd && Vc * Vc > Ad * Ad )    // Phase de decelaration
    {
      Vc = Vc - Ad * sensD;
      PID_consigne_distance = PID_consigne_distance + Vc;
      freinD = 1;
    }
    else if ((consigne_distance - PID_consigne_distance) * (consigne_distance - PID_consigne_distance) <= Dfd * Dfd && Vc * Vc <= Ad * Ad)    // Phase de finition pour arriver a la valeur voulue
    {
      Vc = 0;
      PID_consigne_distance = consigne_distance;
      finDeplacementDistance = true;
    }

    if (IA_STATE == SLAVE) {                                // Si robot en mode esclave...
      envoi_coordonnees();                                            // ... envoi des coordonnees a l'interface
    }
    pause(1.0 / (double)(frequence_timer_asserv) * 1000.0);     // Attente de 0,02s
  }
}




// GENERATION D'UNE CONSIGNE DE ROTATION :
void tourne(double degres) {
  /*
      Génaration d'une consigne de rotation, d'un angle donnee en argument (en degre), avec un trapeze de vitesse
      PID_consigne_orientation : consigne en orientation envoyee à l'asservissement
      consigne_orientation : orientation a atteindre
  */
  // Initialisation des variables
  boolean freinO = false;                   // Frein en orientation (false : pas de ralentissement ; true : ralentissement)
  vitO_init = 100;                          // Vitesse maximale du robot (en tic / temps de boucle) initialisee
  vitO = vitO_init;                         // Vitesse courante du robot (en tic / temps de boucle)
  int Ao = 3;                               // Consigne d'acceleration du robot (en tic / temps de boucle²) initialisee
  int Dfo = 0;                              // Distance de freinage necessaire a l'arret du robot (en tic)
  int Wc = 0;                               // Consigne de vitesse du robot (en tic/temps de boucle)
  bool finDeplacementOrientation = false;   // Etat du deplacement (false : robot en deplacement ; true : deplacement fini)

  // Calcul de la consigne
  consigne_orientation = consigne_orientation + (int)(degres / 360.0 * (double)ResEnc * (double)EMPATEMENT / (double)RAYON_ROUE);     // Consigne courante (en tic) = consigne precedente (en tic) + angle demande en argument de la fonction (converti en tic)
  go = true;                                                              // Mise en marche du robot
  if (consigne_orientation - PID_consigne_orientation >= 0) {                 // Si
    sensO = 1;                                                                      // Rotation vers la droite (sens trigo)
  }
  else {                                                                      // Sinon...
    sensO = -1;                                                                     // Rotation vers la gauche (sens horaire)
  }
  // Generation du trapeze de vitesse
  while (finDeplacementOrientation == false) {                            // Tant que le deplacement n'est pas fini...

    if (freinO == false) {                                                      // Si pas en ralentissement...
      Dfo = 0.53 * Wc * Wc / Ao;                                                      // Calcul de la distance de freinage (en tic) pour la vitesse de consigne (le 0.55 sert a empecher les bugs dus a la discretisation)
    }
    if ((consigne_orientation - PID_consigne_orientation) * (consigne_orientation - PID_consigne_orientation) > Dfo * Dfo && Wc * Wc < vitO * vitO)         // Phase d'acceleration
    {
      Wc = Wc + Ao * sensO;
      PID_consigne_orientation = PID_consigne_orientation + Wc;
    }
    else if ((consigne_orientation - PID_consigne_orientation) * (consigne_orientation - PID_consigne_orientation) > Dfo * Dfo && Wc * Wc >= vitO * vitO)   // Phase a vitesse constante
    {
      Wc = vitO * sensO;
      PID_consigne_orientation = PID_consigne_orientation + Wc;
    }
    else if ((consigne_orientation - PID_consigne_orientation) * (consigne_orientation - PID_consigne_orientation) <= Dfo * Dfo && Wc * Wc > Ao * Ao )      // Phase de deceleration
    {
      Wc = Wc - Ao * sensO;
      PID_consigne_orientation = PID_consigne_orientation + Wc;
      freinO = 1;
    }
    else if ((consigne_orientation - PID_consigne_orientation) * (consigne_orientation - PID_consigne_orientation) <= Dfo * Dfo && Wc * Wc <= Ao * Ao)      // Phase de finition pour arriver a la valeure voulue
    {
      Wc = 0;
      PID_consigne_orientation = consigne_orientation;
      finDeplacementOrientation = true;
    }

    if (IA_STATE == SLAVE) {                                // Si robot en mode esclave...
      envoi_coordonnees();                                            // ... envoi des coordonnees a l'interface
    }
    pause(1 / (double)(frequence_timer_asserv) * 1000);     // Attente de 0,02s
  }

}




// ASSERVISSEMENT DU ROBOT
void Asservissement() {
  // Asservissement en distance
  erreur_distance_old = erreur_distance;                                   // Stockage de l'erreur courante dans l'erreur precedente
  erreur_distance = PID_consigne_distance - distance_parcourue_tic;        // Calcul de la nouvelle erreur
  derivee_distance = erreur_distance - erreur_distance_old;                // Calcul de la derivee de la nouvelle erreur
  PWM_distance = DkP * erreur_distance + DkD * derivee_distance;           // Calcul de la commande proportionnelle derivee pour la consigne en distance
  if (go == true && j < TAILLE_MAX) {                                      // Calcul des tableaux pour matlab
    PID__distance[j] = PWM_distance;
    consigne__distance[j] = PID_consigne_distance;
  }

  // Asservissement en orientation
  erreur_orientation_old = erreur_orientation;                              // Stockage de l'erreur courante dans l'erreur precedente
  erreur_orientation = PID_consigne_orientation - orientation_tic;          // Calcul de la nouvelle erreur
  derivee_orientation = erreur_orientation - erreur_orientation_old;        // Calcul de la derivee de l'erreur
  PWM_orientation = OkP * erreur_orientation + OkD * derivee_orientation;   // Calcul de la commande proportionnelle derivee pour la consigne en orientation
  if (go == true && j < TAILLE_MAX) {                                       // Calcul des tableaux pour matlab
    PID__orientation[j] = PWM_orientation;
  }

  // Generation de la consigne en tension pour moteur droit
  PWM_droit = PWM_distance + PWM_orientation;                               // Calcul de la consigne
  if (PWM_droit < Vmin && PWM_droit > (-1 * Vmin)) {                        // Si la valeur absolue de la consigne est plus petite que la tension minimale moteur...
    PWM_droit = 0;                                                               // Consigne nulle : seuillage pour eviter d'envoyer une commande trop faible et que le moteur ne tourne pas (risque de surchauffe)
  }
  if (PWM_droit < (-1 * Vmax)) {                                            // Si la consigne est plus petite que l'oppose de la tension maximale moteur...
    PWM_droit = (-1 * Vmax);                                                     // Valeur absolue de la consigne egale a la tension maximale : ecretage de la commande en PWM pour tension negative
  }
  else if (PWM_droit >  Vmax) {                                             // Si la consigne est plus grande que la tension maximale moteur...
    PWM_droit = Vmax;                                                            // Consigne egale a la tension maximale : ecretage de la commande en PWM pour tension positive
  }
  if (go == true && j < TAILLE_MAX) {                                       // Calcul des tableaux pour matlab
    PWM__droite[j] = PWM_droit;
  }

  // Generation de la consigne en tension pour moteur gauche
  PWM_gauche = PWM_distance - PWM_orientation;                               // Calcul de la consigne
  if (PWM_gauche < Vmin && PWM_gauche > (-1 * Vmin)) {                       // Si la valeur absolue de la consigne est plus petite que la tension minimale moteur...
    PWM_gauche = 0;                                                               // Consigne nulle : seuillage pour eviter d'envoyer une commande trop faible et que le moteur ne tourne pas (risque de surchauffe)
  }
  if (PWM_gauche < (-1 * Vmax)) {                                            // Si la consigne est plus petite que l'oppose de la tension maximale moteur...
    PWM_gauche = (-1 * Vmax);                                                     // Valeur absolue de la consigne egale a la tension maximale : ecretage de la commande en PWM pour tension negative
  }
  else if (PWM_gauche >  Vmax) {                                             // Si la consigne est plus grande que la tension maximale moteur...
    PWM_gauche = Vmax;                                                            // Consigne egale a la tension maximale : ecretage de la commande en PWM pour tension positive
  }
  if (go == true && j < TAILLE_MAX) {                                        // Calcul des tableaux pour matlab
    PWM__gauche[j] = PWM_gauche;
    consigne__orientation[j] = PID_consigne_orientation;
  }
}



// GENERATION DE TRAJECTOIRE TYPE AIM AND SHOOT :
void aimShoot(int x, int y) {
  // AIM :
  double consigne_orientation_brute = atan2(y - ODO_Y, x - ODO_X) - ODO_Theta;                  // Calcul de la distance en orientation a parcourir

  // Recalcul de la distance en orientation a parcourir entre -pi et pi
  if (consigne_orientation_brute > PI) {
    consigne_orientation_brute = consigne_orientation_brute - 2 * PI;
  }
  else if (consigne_orientation_brute < -PI) {
    consigne_orientation_brute = consigne_orientation_brute + 2 * PI;
  }
  consigne_orientation_brute = (int)(consigne_orientation_brute * 360 / 2.0 / PI);

  tourne(consigne_orientation_brute);                                                           // Le robot tourne de la distance en orientation a parcourir

  // SHOOT :
  int distance_avance = (int)(sqrt( (x - ODO_X) * (x - ODO_X) + (y - ODO_Y) * (y - ODO_Y)) * (double)ResEnc / (2 * PI * (double)RAYON_ROUE)) ;                  // Calcul de la distance a parcourir
  avance(distance_avance);                                                                      // Le robot avance de la distance a parcourir

}



// GENERATION D'UNE POSITION ANGULAIRE ABSOLUE (angle donne en argument (en degre)):
void orientationAbsolue(double degres) {
  double delta_theta = degres * 2 * PI / 360.0 - ODO_Theta;     // Difference (en radian) entre la position angulaire courante et celle demandee
  // Calcul de la difference entre -pi et pi :
  if (delta_theta > PI) {                                       // Si la difference est superieure a pi...
    delta_theta = delta_theta - 2 * PI;                                 // Difference retranchee de 2 pi
  }
  else if (delta_theta < -PI) {                                 // Si la difference est inferiere a -pi...
    delta_theta = delta_theta + 2 * PI;                                 // Difference augmentee de 2 pi
  }
  tourne(delta_theta * 360 / 2.0 / PI);                         // Le robot tourne de la difference (convertie en degre pour utiliser la fonction tourne)
}



// CALLAGE AVANT :
void callageAV(int timeout) {                                     // Prend en argument le temps limite pour effectuer le callage arriere
  go = false;                                                     // Arret du robot
  ds_callage = 0;                                                 // Initialisation de la distance parcourue entre la consigne de callage et le callage
  digitalWrite(DIRD, LOW);
  digitalWrite(DIRD, HIGH);
  digitalWrite(DIRG, HIGH);
  cfcAvGstate = false;                                            // Initialisation du capteur fin de course avant gauche
  cfcAvDstate = false;                                            // Initialisation du capteur fin de course avant droit
  long t_start = millis();                                        // t_start devient la duree (en milliseconde) ecoulee depuis le demarrage de l'execution du programme
  long t = millis();                                              // t devient la duree (en milliseconde) ecoulee depuis le demarrage de l'execution du programme
  while ((cfcAvGstate == false) || (cfcAvDstate == false)) {      // Tant que aucun des capteurs fin de course avant n'est active...
    cfcAvGstate = digitalRead(cfcAvG);                                 // Verification de l'etat du capteur fin de course avant gauche
    cfcAvDstate = digitalRead(cfcAvD);                                 // Verification de l'etat du capteur fin de course avant droit
    moteurD(Vmin + 10);                                                // Moteur droit recoit une consigne pour petite vitesse en marche avant
    moteurG(Vmin + 10);                                                // Moteur gauche recoit une consigne pour petite vitesse en marche avant
    delayMicroseconds(100);                                            // Attente de 100 microsecondes
    t = millis();                                                      // t devient la duree (en milliseconde) ecoulee depuis le demarrage de l'execution du programme
    if ((t - t_start) > timeout) {                                     // Si le temps ecoule depuis l'entree dans la fonction est superieur au temps limite pour effectuer le callage (donne en argument)...
      break;                                                                // Sortir de la boucle
    }
    if (IA_STATE == SLAVE) {                                           // Si utilisation du robot en mode slave...
      envoi_coordonnees();                                                  // Envoi des coordonnees du robot a l'interface
    }
  }
  moteurD(0);                                                       // Consigne moteur droit nulle
  moteurG(0);                                                       // Consigne moteur gauche nulle

  PID_consigne_distance += ds_callage;                             // Prise en compte du callage dans l'asservissement
  consigne_distance += ds_callage;                                 // Prise en compte du callage dans la consigne

  pause(200);                                                      // Attente de 0,2 s
}



// CALLAGE ARRIERE :
void callageAR(int timeout) {                                     // Prend en argument le temps limite pour effectuer le callage arriere
  go = false;                                                     // Arret du robot
  ds_callage = 0;                                                 // Initialisation de la distance parcourue entre la consigne de callage et le callage
  cfcArGstate = false;                                            // Initialisation du capteur fin de course arriere gauche
  cfcArDstate = false;                                            // Initialisation du capteur fin de course arriere droit
  long t_start = millis();                                        // t_start devient la duree (en milliseconde) ecoulee depuis le demarrage de l'execution du programme
  long t = millis();                                              // t devient la duree (en milliseconde) ecoulee depuis le demarrage de l'execution du programme
  while ((cfcArGstate == false) || (cfcArDstate == false)) {      // Tant que aucun des capteurs fin de course arriere n'est active...
    cfcArGstate = digitalRead(cfcArG);                                    // Verification de l'etat du capteur fin de course arriere gauche
    cfcArDstate = digitalRead(cfcArD);                                    // Verification de l'etat du capteur fin de course arriere droit
    moteurD(-(Vmin + 10));                                                // Moteur droit recoit une consigne pour petite vitesse en marche arriere
    moteurG(-(Vmin + 10));                                                // Moteur gauche recoit une consigne pour petite vitesse en marche arriere
    delayMicroseconds(100);                                               // Attente de 100 microsecondes
    t = millis();                                                         // t devient la duree (en milliseconde) ecoulee depuis le demarrage de l'execution du programme
    if ((t - t_start) > timeout) {                                        // Si le temps ecoule depuis l'entree dans la fonction est superieur au temps limite pour effectuer le callage (donne en argument)...
      break;                                                                      // Sortir de la boucle
    }
    if (IA_STATE == SLAVE) {                                              // Si utilisation du robot en mode slave...
      envoi_coordonnees();                                                        // Envoi des coordonnees du robot a l'interface
    }
  }
  moteurD(0);                                                       // Consigne moteur droit nulle
  moteurG(0);                                                       // Consigne moteur gauche nulle

  PID_consigne_distance += ds_callage ;                             // Prise en compte du callage dans l'asservissement
  consigne_distance += ds_callage;                                  // Prise en compte du callage dans la consigne

  pause(200);                                                       // Attente de 0,2 s
}



// REDEFINITION DE LA COORDONNEE X (argument en mm) :
void setODOX(int x) {
  ODO_X = x;
}



// REDEFINITION DE LA COORDONNEE Y (argument en mm) :
void setODOY(int y) {
  ODO_Y = y;
}



// REDEFINITION DE L'ORIENTATION (argument en degre):
void setODOTheta(int angle) {
  ODO_Theta = angle * 2 * PI / 360.0;       // Conversion en radians
}



// LED RVB ALLUMEE EN VERT :
void ledverte() {
  analogWrite(ledR, 0);
  analogWrite(ledV, 128);
  analogWrite(ledB, 0);
}



// LED RVB ALLUMEE EN ORANGE :
void ledorange() {
  analogWrite(ledR, 255);
  analogWrite(ledV, 165);
  analogWrite(ledB, 0);
}



// LED RVB ALLUMEE EN ROSE :
void ledrose() {
  analogWrite(ledR, 128);
  analogWrite(ledV, 0);
  analogWrite(ledB, 128);
}



// LED RVB ETEINTE :
void ledeteinte() {
  analogWrite(ledR, 0);
  analogWrite(ledV, 0);
  analogWrite(ledB, 0);
}



// ENREGISTREMENT DE LA COULEUR DE L'EQUIPE :
void tirette() {
  bool state_tirette = true;                    // Etat du capteur fin de course de la tirette (true : tirette en place ; false : tirette retiree)
  while (state_tirette == true) {               // Tant que la tirette est en place...
    couleur = digitalRead(SWITCH_COULEUR);            // Stockage dans la variable couleur la couleur de l'interrupteur couleur
    if (couleur == ORANGE) {                          // Si equipe orange...
      ledorange();                                            // LED RVB allumee en orange
    }
    if (couleur == VERT) {                            // Si equipe verte...
      ledverte();                                             // LED RVB allumee en vert
    }
    state_tirette = digitalRead(SWITCH_TIRETTE);      // Prise en compte de l'etat du capteur fin de course de la tirette
    delayMicroseconds(10000);                         // Attendre 0,1 seconde
  }
}



// COMPORTEMENT DU ROBOT A LA FIN DU MATCH (declenchement par Timer7) :
void finmatch() {
  go = false;                             // Arret du robot
  moteurD(0);                             // Consigne moteur droit nulle
  moteurG(0);                             // Consigne moteur gauche nulle
  Timer1.stop();                          // Arret Timer1
  Timer7.stop();                          // Arret Timer7
  while (1) {                             // Faire indefiniment... (clignotement LED bleue et LED RVB en rose)
    digitalWrite(LED_BLEU, HIGH);                 // LED bleue allumee
    ledeteinte();                                 // LED RVB eteinte
    delayMicroseconds(500 * 1000);                // Attendre 0,5 secondes
    digitalWrite(LED_BLEU, LOW);                  // LED bleue eteinte
    ledrose();                                    // LED RVB allumee en rose
    delayMicroseconds(500 * 1000);                // Attendre 0,5 secondes
  }
}



// EVITEMENT DES OBSTACLES :
void evitement() {
  state_AVD = digitalRead(pin_AVD);                                 // Prise en compte de l'etat de l'US avant droit
  state_AVG = digitalRead(pin_AVG);                                 // Prise en compte de l'etat de l'US avant gauche
  state_AR = digitalRead(pin_AR);                                   // Prise en compte de l'etat de l'US arriere

  if (mode_US == AVEUGLE) {}                                        // Si utilisation des US en mode aveugle, ne rien faire

  else if (mode_US == NORMAL) {                                     // Si utilisation des US en mode normal...
    if ((state_AVD  == 1) || (state_AVG == 1) || (state_AR == 1)) {           // Si au moins un des US est active...
      vitD = 0;                                                                       // Vitesse en distance nulle
      vitO = 0;                                                                       // Vitesse en orientation nulle
    }
    else {                                                                    // Sinon...
      vitD = vitD_init;                                                               // Vitesse en distance maximale
      vitO = vitO_init;                                                               // Vitesse en orientation maximale
    }
  }

  else if (mode_US == INTELLIGENT) {                                // Si utilisation des US en mode intelligent...
    /*
      if( ( (PID_pid > 0) && (PID_pid2 > 0) && ( (state_AVD == 1)||(state_AVG == 1) ) ) || ( (PID_pid < 0) && (PID_pid2 < 0) && ( (state_ARD == 1)||(state_ARG == 1) ) ) ){   // Test en avance ou en recul
          vitD = 0;
          vitO = 0;
      }
      else if( ( (PID_pid < 0) && (PID_pid2 > 0) && ( (state_AVD == 1)||(state_ARG == 1) ) ) || ( (PID_pid < 0) && (PID_pid2 > 0) && ( (state_ARD == 1)||(state_AVG == 1) ) ) ){  // Test rotation droite ou gauche
          vitD = 0;
          vitO = 0;
      }

      else if( ( (PID_pid == 0) && (PID_pid2 == 0) && ( (state_AVD == 1)||(state_AVG == 1)||(state_ARD == 1)||(state_AVG == 1)))){
          vitD = 0;
          vitO = 0;
      }
    */

  }
}

// ACTIONNEURS

void Controle_Bras(String deplacement) {
  if (deplacement == "ouverture") {
    Servo_Bras_D.write(55);                            //ouverture des bras (gauche et droit)
    Servo_Bras_G.write(125);                             //55 car initialisé à 180 (faire un schéma)
    delay(15);
  }


  else if (deplacement == "fermeture") {                  //retour en position initiale
    Servo_Bras_D.write(180);
    Servo_Bras_G.write(0);
    delay(15);
  }


  else if (deplacement == "attraper") {                 //permet de replier les bras pour attraper les palets (80 degrés à vérifier)
    double angle = 80;
    //for (int i = 125; i>angle; i--) {
    Servo_Bras_D.write(180 - 70);
    Servo_Bras_G.write(70);
    delay(15);
  }


  else if (deplacement == "relacher") {                 //permet de relacher les palets avec les bras droits
    Servo_Bras_D.write(90);
    Servo_Bras_G.write(90);
    delay(15);
  }
}

void Controle_Rateau(String deplacement) {
  if (deplacement == "descente") {
    double angle = 90;                                  //angle d'ouverture en degrés (à définir)
    for (int i = 0; i < angle; i++) {                   //inclinaison vers le bas
      Servo_Rateau.write(i);
      delay(15);
    }
  }

  else if (deplacement == "montee") {
    double angle = -90;                                 //angle de fermeture en degrés (techniquement -90 vu qu'on ferme toujours APRES avoir ouvert
    //ATTENTION à ne pas utiliser lorsque le rateau est DEJA en haut sinon on casse tout!!
    for (int i = angle; i > 0; i--) {                 //remontee du rateau
      Servo_Rateau.write(i);
      delay(15);
    }
  }
}

void Controle_Pont(String deplacement) {
  if (deplacement == "descente") {
    double angle = 45;                                 //angle d'ouverture en degrés (à définir)
    for (int i = 0; i < angle; i++) {                  //inclinaison vers le bas
      Servo_Pont.write(i);
      delay(15);
    }
  }

  else if (deplacement == "montee") {
    double angle = -45;                                 //angle de fermeture en degrés (techniquement -45 vu qu'on ferme toujours APRES avoir ouvert
    //ATTENTION à ne pas utiliser lorsque le pont est DEJA en haut sinon on casse tout!!
    for (int i = angle; i > 0; i--) {                 //remontee du pont
      Servo_Pont.write(i);
      delay(15);
    }
  }
}
