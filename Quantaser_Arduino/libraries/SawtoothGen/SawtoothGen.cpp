#include <SawtoothGen.h> 


SawtoothGen::SawtoothGen(uint8_t integrator_sense, uint8_t charge_ctrl) 
{
	p_outputsense = integrator_sense;
	p_chargecontrol = charge_ctrl;
	
//	pinMode(p_outputsense, INPUT);
	pinMode(p_chargecontrol, OUTPUT);
	digitalWrite(p_chargecontrol, LOW); // initial charge C
}
void SawtoothGen::Sawtooth_out() 
{
	int AI_read;
	
	AI_read = analogRead(p_outputsense);	
//	Serial.println(AI_read); 
	if(AI_read >= AI_MAX) digitalWrite(p_chargecontrol,HIGH); // discharge C					
	if(AI_read <= AI_MIN) digitalWrite(p_chargecontrol,LOW); // charge C

}

