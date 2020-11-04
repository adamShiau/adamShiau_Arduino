#ifndef AD5541_H
#define AD5541_H
class AD5541 {
private:
	unsigned char _adcsel;
public:
	AD5541();
	void init();
	void SetPin(unsigned char pin);
	void ModeWrite(unsigned int dacvalue);
	void NormalWrite(unsigned int dacvalue);

};
#endif
