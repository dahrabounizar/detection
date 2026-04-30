// =============================================================
//  PONT ROULANT 3 AXES — ZV (X/Y) + TRAPÉZOÏDAL CLASSIQUE (Z)
//  AVEC ROTATION DYNAMIXEL + AIMANT (LOGIQUE STANDARD)
//  MODIFICATIONS INTÉGRÉES : 
//  - Anti-rebond (debounce) logiciel sur le Homing X/Y.
//  - Logique Z Inversée : Z>0 raccourcit le câble.
//  - Rayon poulie Z dynamique (+1mm / tour).
//  - Filtrage auto de "OCX" dans les commandes série.
//  - Couple de maintien Z actif à l'arrêt.
//  - Désactivation du couple Z pendant 1s à l'initialisation.
//  - Initialisation Dynamixel à 50° par rapport à la référence.
//  - MODIF : Capteurs de fin de course actifs UNIQUEMENT pendant l'initialisation.
// =============================================================

#include <Arduino.h>
#include <MX_AX_Serial1.h>

// ================== DYNAMIXEL (Rotation) =====================
const int DXL_ID = 2;                 // ID du moteur Dynamixel
const int DXL_SPEED = 50;             // Vitesse de rotation
const float DXL_ZERO_OFFSET = 160.0f; // Zéro logique correspond à 160° physique
float current_angle = 0.0f;           // Angle logique courant (degrés)

// ================== PINS AXE X ===============================
#define X_STEP    7
#define X_DIR     5
#define X_ENABLE  6
#define X_FC      33   // Fin de course X (actif LOW)

// ================== PINS AXE Y ===============================
#define Y_STEP    8
#define Y_DIR     9
#define Y_ENABLE  10
#define Y_FC      53  // Fin de course Y (actif LOW)

// ================== PINS AXE Z ===============================
#define Z_STEP    11
#define Z_DIR     13
#define Z_ENABLE  3
#define Z_FC      A11    // Fin de course Z (actif LOW)

// ================== PIN AIMANT ================================
#define AIMANT_PIN 12  // Électroaimant (relais/transistor)

// ================== DIRECTIONS LOGIQUES ======================
#define X_DIR_NEG LOW    // Vers le capteur X
#define X_DIR_POS HIGH   // S'éloigne du capteur X
#define Y_DIR_NEG LOW    // S'éloigne du capteur Y
#define Y_DIR_POS HIGH   // Vers le capteur Y
#define Z_DIR_VERS_CAPTEUR  HIGH  // Monter = enrouler le câble
#define Z_DIR_ELOIGNE       LOW   // Descendre = dérouler le câble

// ================== PARAMÈTRES PHYSIQUES =====================
const float g            = 9.81f;
const float L_CABLE_INIT = 0.80f;    // Longueur du câble après homing (80 cm)
const float r_pulley_xy  = 0.005f;   // Rayon fixe de poulie X/Y (5 mm)

// --- Rayon Z Dynamique ---
const float r_pulley_z_base = 0.0145f;  // Rayon fixe initial de poulie Z (14.5 mm)
const float dr_per_turn     = 0.001f;   // Augmentation de 1 mm par tour
float current_r_pulley_z    = r_pulley_z_base; 

const float L_FIL        = 0.60f;    // [m] longueur fil de référence
const float H_CYL        = 0.150f;   // [m] hauteur de la charge cylindrique

// ================== LIMITES ET HOMING ========================
const float LIMIT_X_MAX     = 0.71f;
const float LIMIT_Y_MAX     = 0.37f;
const float LIMIT_Z_MAX     = 0.60f;  
const float V_HOMING        = 0.03f;  // Vitesse de homing
const float DIST_RETRAIT_XY = 0.01f;  // 1 cm de retrait pour X et Y

// Positions courantes
float current_pos_x = 0.0f;
float current_pos_y = 0.0f;
float current_pos_z = 0.0f;  
bool  is_homed      = false;

// ================== MOTEUR ===================================
const long  stepsPerRev   = 200;
const long  microsteps    = 8;
const long  N             = stepsPerRev * microsteps; // 1600 pas/tour
const float m_per_step_xy = (TWO_PI * r_pulley_xy) / (float)N;  

// Filtre ZV partagé X/Y 
float ZV_DELAY_S        = 0.0f;       
const float ZV_AMP1     = 0.5f;
const float ZV_AMP2     = 0.5f;

// ================== PROFIL XY ================================
float xy_dist, xy_v_max, xy_acc;
float xy_t_acc, xy_t_flat, xy_t_total;
float cos_theta, sin_theta;  

// ================== PROFIL Z =================================
const float Z_SPEED = 0.10f;  
const float Z_ACCEL = 0.10f;  
float z_dist, z_v_max;
float z_t_acc, z_t_flat, z_t_total;
int   z_dir_state;
long  z_steps_target = 0;
long  z_steps_done   = 0;

// ================== ÉTAT MACHINE =============================
enum State { IDLE, RUNNING, HOMING };
State currentState = IDLE;

unsigned long t_start_us, last_ctrl_us;
const unsigned long CTRL_DT_US = 1000;  // 1 ms

unsigned long stepPeriod_X, stepPeriod_Y, stepPeriod_Z;
unsigned long lastStep_X, lastStep_Y, lastStep_Z;

bool xy_done = false;
bool z_done  = false;

// ================== FONCTIONS UTILITAIRES ====================

void recalculerZV(float L_cable_total) {
  float L_eff = L_cable_total + (H_CYL / 2.0f);
  if (L_eff < 0.02f) L_eff = 0.02f;  
  ZV_DELAY_S = PI * sqrt(L_eff / g);
}

float evaluerVitesseTrapeze(float t, float t_acc, float t_flat, float t_total, float v_max, float acc) {
  if (t < 0.0f || t > t_total) return 0.0f;
  if (t < t_acc) return acc * t;
  else if (t < (t_acc + t_flat)) return v_max;
  else {
    float dt = t - (t_acc + t_flat);
    return v_max - (acc * dt);
  }
}

unsigned long vitesseToPeriode(float v_m_s, float mps) {
  if (v_m_s < 0.0001f) return 999999UL;
  float sps = v_m_s / mps;
  return (unsigned long)(1000000.0f / sps);
}

void mouvementRetrait(int stepPin, int dirPin, uint8_t dirLevel, float mps, float distance_retrait) {
  digitalWrite(dirPin, dirLevel);
  long steps = (long)(distance_retrait / mps);
  for (long i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(800);
  }
}

void calculerProfilTrapeze(float dist, float v_demande, float a,
                           float &out_v_max, float &out_t_acc, float &out_t_flat, float &out_t_total) {
  float dist_acc = (v_demande * v_demande) / (2.0f * a);
  if (2.0f * dist_acc > dist) {
    out_v_max = sqrt(a * dist);
    out_t_acc = out_v_max / a;
    out_t_flat = 0.0f;
  } else {
    out_v_max = v_demande;
    out_t_acc = v_demande / a;
    out_t_flat = (dist - 2.0f * dist_acc) / v_demande;
  }
  out_t_total = 2.0f * out_t_acc + out_t_flat;
}

// ================== SÉQUENCE DE HOMING =======================
void effectuerHoming() {
  Serial.println("=== HOMING (X/Y) & INIT (Z/DYNAMIXEL) ===");
  currentState = HOMING;
  
  // --- DÉSACTIVATION DU COUPLE Z PUIS ACTIVATION APRÈS 1 SECONDE ---
  Serial.println("Desactivation du couple Z pendant 1 seconde...");
  digitalWrite(Z_ENABLE, HIGH); // Moteur libre (tombe par gravité)
  delay(1000);                  // Attente de 1 seconde
  
  digitalWrite(X_ENABLE, LOW);
  digitalWrite(Y_ENABLE, LOW);
  digitalWrite(Z_ENABLE, LOW);  // Réactivation du couple Z

  unsigned long period_homing_xy = vitesseToPeriode(V_HOMING, m_per_step_xy);
  unsigned long now;

  Serial.println("Phase 1 : Approche X + Y avec Anti-Rebond...");
  digitalWrite(X_DIR, X_DIR_NEG);
  digitalWrite(Y_DIR, Y_DIR_POS); 
  lastStep_X = micros();
  lastStep_Y = micros();

  bool x_hit = (digitalRead(X_FC) == LOW);
  bool y_hit = (digitalRead(Y_FC) == LOW);

  while (!x_hit || !y_hit) {
    now = micros();
    
    if (!x_hit && (now - lastStep_X >= period_homing_xy)) {
      digitalWrite(X_STEP, HIGH); delayMicroseconds(2); digitalWrite(X_STEP, LOW);
      lastStep_X = now;
      if (digitalRead(X_FC) == LOW) {
        delayMicroseconds(200); // Filtre anti-rebond logiciel
        if (digitalRead(X_FC) == LOW) x_hit = true;
      }
    }
    
    if (!y_hit && (now - lastStep_Y >= period_homing_xy)) {
      digitalWrite(Y_STEP, HIGH); delayMicroseconds(2); digitalWrite(Y_STEP, LOW);
      lastStep_Y = now;
      if (digitalRead(Y_FC) == LOW) {
        delayMicroseconds(200); // Filtre anti-rebond logiciel
        if (digitalRead(Y_FC) == LOW) y_hit = true;
      }
    }
  }

  Serial.println("Phase 2 & 3 : Retrait et Z conserve sa position actuelle");
  mouvementRetrait(X_STEP, X_DIR, X_DIR_POS, m_per_step_xy, DIST_RETRAIT_XY);
  mouvementRetrait(Y_STEP, Y_DIR, Y_DIR_NEG, m_per_step_xy, DIST_RETRAIT_XY); 

  Serial.println("Phase 4 : Initialisation Dynamixel");
  // MODIFICATION ICI : Initialisation à 50 degrés par rapport à la référence (160°)
  float init_logical_angle = 50.0f;
  float init_physical_angle = init_logical_angle + DXL_ZERO_OFFSET;
  int dxl_home_pos = (int)((init_physical_angle * 4095.0f) / 360.0f);
  
  Dynamixel.moveSpeed(DXL_ID, dxl_home_pos, DXL_SPEED);
  current_angle = init_logical_angle; // On enregistre 50° comme position actuelle

  current_pos_x = 0.0f;
  current_pos_y = 0.0f;
  current_pos_z = 0.0f; 
  current_r_pulley_z = r_pulley_z_base; // Réinitialisation du rayon
  is_homed = true;
  currentState = IDLE;

  recalculerZV(L_CABLE_INIT);

  digitalWrite(X_ENABLE, HIGH);
  digitalWrite(Y_ENABLE, HIGH);
  digitalWrite(Z_ENABLE, LOW); // Maintien du couple pour Z

  Serial.println("=== ORIGINE DEFINIE ===");
}

// ================== PRÉPARATION MOUVEMENT 3D =================
void preparerMouvement3D(String cmd) {
  int sp1 = cmd.indexOf(' ');
  int sp2 = cmd.indexOf(' ', sp1 + 1);
  int sp3 = cmd.indexOf(' ', sp2 + 1);
  int sp4 = cmd.indexOf(' ', sp3 + 1);
  if (sp1 < 0 || sp2 < 0 || sp3 < 0 || sp4 < 0) {
    Serial.println("FORMAT: x y z vitesse_xy acceleration_xy");
    return;
  }

  float target_x = cmd.substring(0, sp1).toFloat();
  float target_y = cmd.substring(sp1 + 1, sp2).toFloat();
  float target_z = cmd.substring(sp2 + 1, sp3).toFloat();
  float v_xy     = cmd.substring(sp3 + 1, sp4).toFloat();
  float a_xy     = cmd.substring(sp4 + 1).toFloat();

  target_y = target_y * -1.0f;

  target_x = constrain(target_x, 0.0f, LIMIT_X_MAX);
  target_y = constrain(target_y, -LIMIT_Y_MAX, 0.0f);
  target_z = constrain(target_z, 0.0f, LIMIT_Z_MAX);

  float dx = target_x - current_pos_x;
  float dy = target_y - current_pos_y;
  float dz = target_z - current_pos_z;

  bool has_xy = (abs(dx) >= 0.001f || abs(dy) >= 0.001f);
  bool has_z  = (abs(dz) >= 0.001f);

  if (!has_xy && !has_z) return;

  current_pos_x = target_x;
  current_pos_y = target_y;
  current_pos_z = target_z;

  // CORRECTION Z : Z positif = le câble diminue = on enroule
  float L_cable_new = L_CABLE_INIT - target_z; 
  if (L_cable_new < 0.02f) L_cable_new = 0.02f;  
  recalculerZV(L_cable_new);

  Serial.print("L_cable = "); Serial.print(L_cable_new, 3);
  Serial.print(" m | ZV_delay = "); Serial.print(ZV_DELAY_S * 1000.0f, 1); Serial.println(" ms");

  xy_done = true;
  if (has_xy) {
    xy_done = false;
    digitalWrite(X_DIR, dx >= 0.0f ? X_DIR_POS : X_DIR_NEG);
    digitalWrite(Y_DIR, dy >= 0.0f ? Y_DIR_POS : Y_DIR_NEG);
    delay(2);

    xy_dist = sqrt(dx * dx + dy * dy);
    float D_xy = xy_dist;
    cos_theta = (D_xy > 0.0001f) ? abs(dx) / D_xy : 0.0f;
    sin_theta = (D_xy > 0.0001f) ? abs(dy) / D_xy : 0.0f;

    if (a_xy <= 0) a_xy = 0.5f;
    xy_acc = a_xy;
    calculerProfilTrapeze(xy_dist, v_xy, a_xy, xy_v_max, xy_t_acc, xy_t_flat, xy_t_total);
  }

  z_done = true;
  if (has_z) {
    z_done = false;
    z_steps_done = 0;
    
    if (dz > 0) { // Z augmente -> Enroulement -> Rayon augmente
      z_dir_state = Z_DIR_VERS_CAPTEUR;  
    } else {      // Z diminue -> Déroulement -> Rayon diminue
      z_dir_state = Z_DIR_ELOIGNE;       
    }
    digitalWrite(Z_DIR, z_dir_state);
    
    z_dist = abs(dz);
    z_steps_target = 0;
    float d_calc = 0.0f;
    float r_sim = current_r_pulley_z;
    
    // Calcul dynamique du nombre de pas en tenant compte du rayon variable
    while (d_calc < z_dist) {
      d_calc += (TWO_PI * r_sim) / (float)N;
      if (z_dir_state == Z_DIR_VERS_CAPTEUR) r_sim += (dr_per_turn / (float)N);
      else r_sim -= (dr_per_turn / (float)N);
      z_steps_target++;
    }

    calculerProfilTrapeze(z_dist, Z_SPEED, Z_ACCEL, z_v_max, z_t_acc, z_t_flat, z_t_total);
  }

  if (has_xy) { digitalWrite(X_ENABLE, LOW); digitalWrite(Y_ENABLE, LOW); }
  if (has_z)  { digitalWrite(Z_ENABLE, LOW); }

  t_start_us   = micros();
  last_ctrl_us = micros();
  lastStep_X   = micros();
  lastStep_Y   = micros();
  lastStep_Z   = micros();
  stepPeriod_X = 100000UL;
  stepPeriod_Y = 100000UL;
  stepPeriod_Z = 999999UL;

  currentState = RUNNING;
  Serial.println("--- MOUVEMENT 3D DEMARRE ---");
}

// ================== SETUP ====================================
void setup() {
  Serial.begin(115200);

  pinMode(X_STEP, OUTPUT); pinMode(X_DIR, OUTPUT); pinMode(X_ENABLE, OUTPUT);
  pinMode(X_FC, INPUT_PULLUP);
  pinMode(Y_STEP, OUTPUT); pinMode(Y_DIR, OUTPUT); pinMode(Y_ENABLE, OUTPUT);
  pinMode(Y_FC, INPUT_PULLUP);
  pinMode(Z_STEP, OUTPUT); pinMode(Z_DIR, OUTPUT); pinMode(Z_ENABLE, OUTPUT);
  pinMode(Z_FC, INPUT_PULLUP);

  pinMode(AIMANT_PIN, OUTPUT);
  digitalWrite(AIMANT_PIN, LOW); 

  Dynamixel.begin(1000000, 16);
  delay(500);
  
  // MODIFICATION ICI : Initialisation du setup à 50 degrés par rapport à la référence
  float init_logical_angle = 50.0f;
  float init_physical_angle = init_logical_angle + DXL_ZERO_OFFSET;
  int dxl_init_pos = (int)((init_physical_angle * 4095.0f) / 360.0f);
  
  Dynamixel.moveSpeed(DXL_ID, dxl_init_pos, DXL_SPEED);
  current_angle = init_logical_angle; // On initialise la variable à 50.0f

  digitalWrite(X_ENABLE, HIGH);
  digitalWrite(Y_ENABLE, HIGH);
  
  // --- DÉSACTIVATION DU COUPLE Z PUIS ACTIVATION APRÈS 1 SECONDE ---
  Serial.println("Initialisation Z : Desactivation du couple...");
  digitalWrite(Z_ENABLE, HIGH); // Moteur libre
  delay(1000);                  // Attente de 1 seconde
  digitalWrite(Z_ENABLE, LOW);  // Z est verrouille
  Serial.println("Initialisation Z : Couple active.");

  recalculerZV(L_CABLE_INIT);

  Serial.println("=== PONT ROULANT 3 AXES ===");
}

// ================== LOOP =====================================
void loop() {

  if (currentState == IDLE && Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.replace("OCX", ""); // FILTRE : Efface "OCX" de la commande
    
    if (input.equalsIgnoreCase("init")) {
      effectuerHoming();
    } else if (input.equalsIgnoreCase("aimant_on")) {
      digitalWrite(AIMANT_PIN, HIGH); 
      Serial.println("AIMANT_ON");
    } else if (input.equalsIgnoreCase("aimant_off")) {
      digitalWrite(AIMANT_PIN, LOW);  
      Serial.println("AIMANT_OFF");
    } else if (input.equalsIgnoreCase("z_hold_off")) {
      digitalWrite(Z_ENABLE, HIGH);
      Serial.println("ATTENTION : Frein Z desactive.");
    } else if (input.equalsIgnoreCase("z_hold_on")) {
      digitalWrite(Z_ENABLE, LOW);
      Serial.println("FREIN Z : Maintien reactive.");
    } else if (input.startsWith("rot ")) {
      float logical_angle = input.substring(4).toFloat();
      float physical_angle = logical_angle + DXL_ZERO_OFFSET;
      
      if (physical_angle < 0.0f) physical_angle = 0.0f;
      if (physical_angle > 360.0f) physical_angle = 360.0f;
      
      int pos = (int)((physical_angle * 4095.0f) / 360.0f);
      if (pos < 0) pos = 0;
      if (pos > 4095) pos = 4095;
      
      Dynamixel.moveSpeed(DXL_ID, pos, DXL_SPEED);
      current_angle = logical_angle; 
      Serial.print("ROT_OK | Degres: "); Serial.println(logical_angle, 1);
      
    } else if (is_homed && input.length() > 0) {
      preparerMouvement3D(input);
    } else if (!is_homed && input.length() > 0) {
      Serial.println("ERREUR: Tapez 'init' d'abord !");
    }
  }

  // --- LE BLOC SÉCURITÉ EN MOUVEMENT A ÉTÉ SUPPRIMÉ ICI ---
  // (La machine ignorera désormais les capteurs pendant les déplacements normaux)

  unsigned long now = micros();

  // --- BOUCLE DE CONTRÔLE (1 ms) ---
  if (currentState == RUNNING && (now - last_ctrl_us >= CTRL_DT_US)) {
    last_ctrl_us = now;
    float t = (float)(now - t_start_us) / 1000000.0f;

    // === Contrôle XY (ZV) ===
    if (!xy_done) {
      float v1 = evaluerVitesseTrapeze(t,              xy_t_acc, xy_t_flat, xy_t_total, xy_v_max, xy_acc);
      float v2 = evaluerVitesseTrapeze(t - ZV_DELAY_S, xy_t_acc, xy_t_flat, xy_t_total, xy_v_max, xy_acc);
      float v_zv = (ZV_AMP1 * v1) + (ZV_AMP2 * v2);

      if (t > (xy_t_total + ZV_DELAY_S) && v_zv < 0.0005f) {
        xy_done = true;
      } else {
        stepPeriod_X = vitesseToPeriode(v_zv * cos_theta, m_per_step_xy);
        stepPeriod_Y = vitesseToPeriode(v_zv * sin_theta, m_per_step_xy);
      }
    }

    // === Contrôle Z (Dynamique) ===
    if (!z_done) {
      float v_z = evaluerVitesseTrapeze(t, z_t_acc, z_t_flat, z_t_total, z_v_max, Z_ACCEL);
      float current_m_per_step_z = (TWO_PI * current_r_pulley_z) / (float)N;
      
      if (z_steps_done >= z_steps_target) {
        z_done = true;
      } else {
        stepPeriod_Z = vitesseToPeriode(v_z, current_m_per_step_z);
      }
    }

    if (xy_done && z_done) {
      digitalWrite(X_ENABLE, HIGH);
      digitalWrite(Y_ENABLE, HIGH);
      digitalWrite(Z_ENABLE, LOW); // Maintien actif
      currentState = IDLE;
      Serial.println("=== ARRIVE ===");
      return;
    }
  }

  // --- GÉNÉRATION D'IMPULSIONS ---
  if (currentState == RUNNING) {
    if (!xy_done && cos_theta > 0.001f && (now - lastStep_X >= stepPeriod_X)) {
      digitalWrite(X_STEP, HIGH); delayMicroseconds(2); digitalWrite(X_STEP, LOW);
      lastStep_X += stepPeriod_X;
    }
    if (!xy_done && sin_theta > 0.001f && (now - lastStep_Y >= stepPeriod_Y)) {
      digitalWrite(Y_STEP, HIGH); delayMicroseconds(2); digitalWrite(Y_STEP, LOW);
      lastStep_Y += stepPeriod_Y;
    }
    if (!z_done && (now - lastStep_Z >= stepPeriod_Z)) {
      digitalWrite(Z_STEP, HIGH); delayMicroseconds(2); digitalWrite(Z_STEP, LOW);
      lastStep_Z += stepPeriod_Z;
      z_steps_done++;
      
      // Mise à jour physique du rayon en temps réel (1 mm par tour)
      if (z_dir_state == Z_DIR_VERS_CAPTEUR) current_r_pulley_z += (dr_per_turn / (float)N);
      else current_r_pulley_z -= (dr_per_turn / (float)N);
    }
  }
}