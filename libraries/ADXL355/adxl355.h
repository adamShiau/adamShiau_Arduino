#ifndef ADXL355_H
#define ADXL355_H

class adxl355
{
	private:
		char p_scl_en;
		
		void p_I2CWriteData(unsigned char, unsigned char);
		unsigned char p_I2CReadData(unsigned char);
		void p_scl_mux_enable(void);
		void p_scl_mux_disable(void);
		
	public:
		adxl355(char);
		void init(void);
		void setRegVal(unsigned char, unsigned char);
		void printRegVal(char [], unsigned char, unsigned char);
		void printRegAll(void);
	
};


#endif