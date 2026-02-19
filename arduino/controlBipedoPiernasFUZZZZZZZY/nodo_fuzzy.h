#include <Fuzzy.h>

Fuzzy *fuzzy = new Fuzzy();

void crearFuzzy(){
  // Creación de entrada
    FuzzyInput *rollError = new FuzzyInput(1);

    // Conjuntos de entrada
    FuzzySet *ENG = new FuzzySet(-0.8,-0.8,-0.393,-0.262);
    rollError->addFuzzySet(ENG);

    FuzzySet *ENM = new FuzzySet(-0.393,-0.262,-0.262,-0.237);
    rollError->addFuzzySet(ENM);

    FuzzySet *ENP = new FuzzySet(-0.262,-0.237,-0.237,-0.005);
    rollError->addFuzzySet(ENP);

    FuzzySet *EZO = new FuzzySet(-0.237,-0.005,0.005,0.237);
    rollError->addFuzzySet(EZO);

    FuzzySet *EPP = new FuzzySet(0.005,0.237,0.237,0.262);
    rollError->addFuzzySet(EPP);

    FuzzySet *EPM = new FuzzySet(0.237,0.262,0.262,0.393);
    rollError->addFuzzySet(EPM);

    FuzzySet *EPG = new FuzzySet(0.262,0.393,0.8,0.8);
    rollError->addFuzzySet(EPG);  

    fuzzy->addFuzzyInput(rollError);
  
  // Creación de salida
    FuzzyOutput *posOut = new FuzzyOutput(1);

    // Conjuntos de salida
    FuzzySet *RNG = new FuzzySet(-0.012,-0.009,-0.009,-0.006);
    posOut->addFuzzySet(RNG);

    FuzzySet *RNM = new FuzzySet(-0.009,-0.006,-0.006,-0.003);
    posOut->addFuzzySet(RNM);

    FuzzySet *RNP = new FuzzySet(-0.006,-0.003,-0.003,0);
    posOut->addFuzzySet(RNP);

    FuzzySet *RZO = new FuzzySet(-0.003,0,0,0.003);
    posOut->addFuzzySet(RZO);

    FuzzySet *RPP = new FuzzySet(0,0.003,0.003,0.006);
    posOut->addFuzzySet(RPP);

    FuzzySet *RPM = new FuzzySet(0.003,0.006,0.006,0.009);
    posOut->addFuzzySet(RPM);

    FuzzySet *RPG = new FuzzySet(0.006,0.009,0.009,0.012);
    posOut->addFuzzySet(RPG);

    fuzzy->addFuzzyOutput(posOut);

  // Construcción de reglas
    // Regla 01
      FuzzyRuleAntecedent *ifErrorNG = new FuzzyRuleAntecedent();
      ifErrorNG->joinSingle(ENG);
      FuzzyRuleConsequent *thenPosRNG = new FuzzyRuleConsequent();
      thenPosRNG->addOutput(RNG);
      FuzzyRule *fuzzyRule01 = new FuzzyRule(1, ifErrorNG, thenPosRNG);
      fuzzy->addFuzzyRule(fuzzyRule01);
    // Regla 02
      FuzzyRuleAntecedent *ifErrorNM = new FuzzyRuleAntecedent();
      ifErrorNM->joinSingle(ENM);
      FuzzyRuleConsequent *thenPosRNM = new FuzzyRuleConsequent();
      thenPosRNM->addOutput(RNM);
      FuzzyRule *fuzzyRule02 = new FuzzyRule(1, ifErrorNM, thenPosRNM);
      fuzzy->addFuzzyRule(fuzzyRule02);
    // Regla 03
      FuzzyRuleAntecedent *ifErrorNP = new FuzzyRuleAntecedent();
      ifErrorNP->joinSingle(ENP);
      FuzzyRuleConsequent *thenPosRNP = new FuzzyRuleConsequent();
      thenPosRNP->addOutput(RNP);
      FuzzyRule *fuzzyRule03 = new FuzzyRule(1, ifErrorNP, thenPosRNP);
      fuzzy->addFuzzyRule(fuzzyRule03);
    // Regla 04
      FuzzyRuleAntecedent *ifErrorZO = new FuzzyRuleAntecedent();
      ifErrorZO->joinSingle(EZO);
      FuzzyRuleConsequent *thenPosRZO = new FuzzyRuleConsequent();
      thenPosRZO->addOutput(RZO);
      FuzzyRule *fuzzyRule04 = new FuzzyRule(1, ifErrorZO, thenPosRZO);
      fuzzy->addFuzzyRule(fuzzyRule04);
    // Regla 05
      FuzzyRuleAntecedent *ifErrorPP = new FuzzyRuleAntecedent();
      ifErrorPP->joinSingle(EPP);
      FuzzyRuleConsequent *thenPosRPP = new FuzzyRuleConsequent();
      thenPosRPP->addOutput(RPP);
      FuzzyRule *fuzzyRule05 = new FuzzyRule(1, ifErrorPP, thenPosRPP);
      fuzzy->addFuzzyRule(fuzzyRule05);
    // Regla 06
      FuzzyRuleAntecedent *ifErrorPM = new FuzzyRuleAntecedent();
      ifErrorPM->joinSingle(EPM);
      FuzzyRuleConsequent *thenPosRPM = new FuzzyRuleConsequent();
      thenPosRPM->addOutput(RPM);
      FuzzyRule *fuzzyRule06 = new FuzzyRule(1, ifErrorPM, thenPosRPM);
      fuzzy->addFuzzyRule(fuzzyRule06);  
    // Regla 07
      FuzzyRuleAntecedent *ifErrorPG = new FuzzyRuleAntecedent();
      ifErrorPG->joinSingle(EPG);
      FuzzyRuleConsequent *thenPosRPG = new FuzzyRuleConsequent();
      thenPosRPG->addOutput(RPG);
      FuzzyRule *fuzzyRule07 = new FuzzyRule(1, ifErrorPG, thenPosRPG);
      fuzzy->addFuzzyRule(fuzzyRule07);     
}

void controlFuzzy(float roll, float altura){
  fuzzyError = hsetPoint - roll;
  fuzzy->setInput(1, fuzzyError);
  fuzzy->fuzzify();
  fuzzyOutput = fuzzy->defuzzify(1);
  static int ciclos4 = 0;
  if(ciclos4<1200){ // ignorar los primeros 1200 ciclos
    ciclos4++;
    fuzzyOutput = 0;
  }
  hRoutput -= fuzzyOutput;
  hLoutput -= fuzzyOutput;

  if (altura + hRoutput > 0.75) {
    hRoutput = 0.75 - altura;    
  } else if (altura + hRoutput < 0.05) {
    hRoutput = 0.05 - altura;    
  }

  if (altura - hLoutput > 0.75) {
    hLoutput = -(0.75 - altura);    
  } else if (altura - hLoutput < 0.05) {
    hLoutput = -(0.05 - altura);    
  }

  if ( (altura + hRoutput) > altura && (altura - hLoutput) > altura){
    hRoutput -= 0.001;
    hLoutput += 0.001;
  }
}