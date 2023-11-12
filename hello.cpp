#include <stdio.h>

struct eeprom_addr{
    int EEPROM_ADDR_A;
    int EEPROM_ADDR_B;
    int EEPROM_ADDR_C;
}ee1, ee2;

void init_eepA(void);
void init_eepB(void);
void print_cont(eeprom_addr *);

int main() {
    init_eepA();
    init_eepB();
    print_cont(&ee2);



    return 0;
}

void print_cont(eeprom_addr *st)
{
    printf("EEPROM_ADDR_A: %d\n", st->EEPROM_ADDR_A);
    printf("EEPROM_ADDR_B: %d\n", st->EEPROM_ADDR_B);
    printf("EEPROM_ADDR_C: %d\n", st->EEPROM_ADDR_C);
}

void init_eepA()
{
    ee1.EEPROM_ADDR_A = 1;
    ee1.EEPROM_ADDR_B = 2;
    ee1.EEPROM_ADDR_C = 3;
}

void init_eepB()
{
    ee2.EEPROM_ADDR_A = 11;
    ee2.EEPROM_ADDR_B = 22;
    ee2.EEPROM_ADDR_C = 33;
}