#ifndef SPARROW_PARA_DEFIN_H
#define SPARROW_PARA_DEFIN_H


#define setSPI_cmd  String("setSPI ")
#define OUTPUT_ADD  String("90 ")
#define OPEN_MODE  String("0")
#define CLOSE_MODE  String("1")

#define GAIN1PWR_ADD  String("8A ")

#define GAIN2PWR_ADD  String("8B ") 

#define MOD_HIGH_ADD  String("81 ")
#define MOD_LOW_ADD  String("82 ")
#define MOD_FREQ_ADD  String("80 ") 
#define MOD_PiVth_ADD  String("8E ") 
#define Polarity_ADD  String("87 ")

#define IGNOR_ADD  String("85 ") 
#define OFFSET_ADD  String("84 ") 
#define StepVth_ADD  String("83 ") 
#define AVG_ADD  String("86 ") 
#define INTER_LIMIT_ADD  String("89 ")
#define RESET_LADDER  String("88 ")


#define readSPI_cmd  String("readSPI ")
#define readTemp_cmd  String("readTemp ")
#define ANGULAR_V  String("17 ")
#define OPEN_V  String("12 ")

#define Read_cmd  String("1")
#define Stop_cmd  String("0")

#define UPPER_BAND_ADD  String("8C ")
#define LOWER_BAND_ADD  String("8D ")

#define MOD_OFF_CMD  String("8F 0")


#endif